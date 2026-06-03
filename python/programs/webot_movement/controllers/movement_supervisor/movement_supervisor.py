"""Webots Supervisor controller for the IMU digital twin.

Reads the live IMU stream (acceleration + gyroscope, 50 Hz) from the physical
Arduino Nano 33 BLE Sense over Bluetooth LE, performs honest dead reckoning
in the supervisor process, and writes the resulting pose into the MobileNano
robot's `translation` and `rotation` fields each tick.

----------------------------------------------------------------------------
About dead reckoning
----------------------------------------------------------------------------
Integrating accelerometer + gyroscope to recover position and orientation is
fundamentally drift-prone: any bias in accel integrates twice to a
quadratically growing position error, any bias in gyro integrates once to a
linearly growing yaw error, and noise integrates to a random walk on top of
that.

For a "slow and steady, simple trajectory" digital-twin demo, four
mitigations make the result watchable for tens of seconds:

  1) Bias calibration. We sit still during the first CALIB_S seconds, average
     each axis, and subtract that mean from every subsequent sample. For az
     the mean includes the ~1 g of gravity, so post-calibration az is the
     "delta from the stationary state" -- exactly what we want to integrate.
  2) Manual drift calibration (optional, via --calibrate-drift SECONDS). The
     2-s static window can only average so much. After the static window,
     this mode integrates freely for tens of seconds, infers the residual
     constant bias from the resulting drift, and persists it to
     drift_calibration.json next to this script. Subsequent normal runs
     auto-load that file and subtract the residual in addition to the
     per-session static bias.
  3) ZUPT (Zero-Velocity Update). When the magnitudes of the bias-corrected
     accel and gyro both stay below a small threshold for several consecutive
     samples, we force the velocity estimate back to zero.
  4) Velocity damping + position clamp. Each tick we apply a small decay to
     velocity, and we softly pull the position back inside the arena if it
     ever drifts past a configurable radius.

----------------------------------------------------------------------------
Assumed physical chip orientation
----------------------------------------------------------------------------
The chip is assumed to be held FLAT during use, with the USB connector
pointed toward the user and the components facing up. In that orientation
the chip-local axes align with the user's frame:

    +chip-X = forward (away from the user)
    +chip-Y = left
    +chip-Z = up    (gravity contributes ~+1 g here when stationary)

If you mount the chip differently (e.g. truly vertical with PCB face forward),
edit the three helpers at the top of `main` -- `forward_accel_g`,
`left_accel_g`, and `yaw_rate_dps` -- to remap which sample component drives
which integrator.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Tuple

from controller import Supervisor  # type: ignore[import-not-found]

from ble_imu import BleImuSource, MockImuSource, ImuSource, Sample

RESIDUAL_CALIBRATION_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "drift_calibration.json",
)

# --- Physical constants ---------------------------------------------------- #
GRAVITY_M_S2 = 9.81

# --- Calibration ---------------------------------------------------------- #
# Sit still for this long while the bias of every IMU axis is averaged.
CALIB_S = 2.0

# --- ZUPT (Zero-Velocity Update) ----------------------------------------- #
# When |corrected accel| (m/s^2) AND |corrected gyro| (deg/s) both stay below
# the thresholds for ZUPT_WINDOW consecutive *new* samples, force velocity
# to zero.
ZUPT_ACCEL_THRESHOLD_M_S2 = 0.15   # ~0.015 g
ZUPT_GYRO_THRESHOLD_DPS = 2.0
ZUPT_WINDOW = 10

# --- Drift control -------------------------------------------------------- #
# Per-tick velocity decay (geometric). At 32 ms tick this is a time constant
# of about 3.2 s, so velocity halves in ~2.2 s of no input.
VELOCITY_DECAY = 0.99

# Output scaling. The integrated velocity is multiplied by SPEED_SCALE
# before being applied to the robot. This does NOT change drift per-meter
# of true motion -- it just zooms out the time axis, so both real motion
# and drift appear at half speed when SPEED_SCALE = 0.5. Useful for keeping
# the robot easy to follow visually and inside the arena longer.
SPEED_SCALE = 0.5
YAW_RATE_SCALE = 0.5   # same idea, but for heading

# Hard cap on horizontal speed (m/s) AFTER SPEED_SCALE is applied, in case
# bias estimation overshoots.
MAX_SPEED_M_S = 0.5

# If the dead-reckoned position drifts further than this from the starting
# point we softly pull it back in (so the robot stays on the visible arena).
MAX_RADIUS_M = 1.4

# --- Low-pass filter on bias-corrected acceleration ---------------------- #
# Single-pole IIR: smoothed = ALPHA * raw + (1 - ALPHA) * smoothed.
# ALPHA = 1.0 disables the filter; ALPHA = 0.2 gives a few-sample average
# without much lag. Smaller values reduce drift from noise but add lag.
ACCEL_LPF_ALPHA = 0.3


# ---------------------------------------------------------------------------- #
# Residual-bias calibration
# ----------------------------------------------------------------------------
# The 2-second static calibration window at startup can only average so much.
# A *manual* drift calibration extends this: hold the chip perfectly still for
# tens of seconds, integrate freely (no ZUPT/decay/clamp/LPF), and back out the
# residual constant bias from the observed quadratic position drift and the
# linear yaw drift.
#
# If `x_drift` is the chip-frame position drift after T seconds of integrating
# from rest, then the constant residual accel bias is
#   db_a = 2 * x_drift / T^2
# and similarly db_gz = yaw_drift / T for gyro.
#
# These corrections are persisted to RESIDUAL_CALIBRATION_PATH so subsequent
# runs can subtract them in addition to the per-session static biases.
# ---------------------------------------------------------------------------- #


def _load_residual_calibration(
    path: str = RESIDUAL_CALIBRATION_PATH,
) -> Optional[Dict[str, Any]]:
    """Return the saved residual-bias dict, or None if missing/invalid."""
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[sup] warning: could not read {path}: {exc}", file=sys.stderr)
        return None
    return {
        "residual_bias_ax_g": float(data.get("residual_bias_ax_g", 0.0)),
        "residual_bias_ay_g": float(data.get("residual_bias_ay_g", 0.0)),
        "residual_bias_gz_dps": float(data.get("residual_bias_gz_dps", 0.0)),
        "calibration_duration_s": float(data.get("calibration_duration_s", 0.0)),
        "calibration_timestamp": data.get("calibration_timestamp", "unknown"),
    }


def _save_residual_calibration(
    residual: Dict[str, Any],
    duration_s: float,
    static_bias_window_s: float,
    path: str = RESIDUAL_CALIBRATION_PATH,
) -> None:
    payload: Dict[str, Any] = {
        "residual_bias_ax_g": residual["ax_g"],
        "residual_bias_ay_g": residual["ay_g"],
        "residual_bias_gz_dps": residual["gz_dps"],
        "calibration_duration_s": duration_s,
        "calibration_timestamp": datetime.now().isoformat(timespec="seconds"),
        "static_bias_window_s": static_bias_window_s,
        "_note": (
            "Auto-generated by movement_supervisor.py --calibrate-drift. "
            "Subtracted from samples in addition to the per-session static "
            "bias on subsequent runs. Re-calibrate if the chip's orientation "
            "or temperature changes substantially (residuals are temperature-"
            "sensitive)."
        ),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def _measure_residual_drift(
    sup: Supervisor,
    source: ImuSource,
    static_biases: Dict[str, float],
    axis_remap: Tuple[
        Callable[[Sample], float],
        Callable[[Sample], float],
        Callable[[Sample], float],
    ],
    duration_s: float,
    timestep_ms: int,
) -> Optional[Dict[str, Any]]:
    """Sit still for `duration_s` and back out residual biases from drift.

    The integration is done in CHIP frame (no rotation into world frame). The
    chip is stationary by assumption, so chip frame is the natural place for
    the residual to live -- it will be subtracted from raw chip-frame samples
    on subsequent runs.

    Returns a dict with keys {ax_g, ay_g, gz_dps, _observed_drift, _duration}
    or None if the simulation was stopped mid-measurement.
    """
    forward_accel_g, left_accel_g, yaw_rate_dps = axis_remap
    print(
        f"[sup] DRIFT CALIBRATION: hold the chip perfectly still for "
        f"{duration_s:.1f} s."
    )
    print(
        "[sup]   (pure integration: ZUPT / velocity decay / position clamp / "
        "LPF all disabled)"
    )
    print(
        "[sup]   Set the chip on a stable surface. Do not touch it, breathe "
        "on it, or bump the table."
    )

    vx_chip = 0.0
    vy_chip = 0.0
    x_chip = 0.0
    y_chip = 0.0
    yaw_deg = 0.0   # integrated gyro_z directly in degrees

    last_seq, _ = source.latest_with_seq()
    last_sample_t = sup.getTime()
    t_start = sup.getTime()
    last_print_t = t_start

    while sup.getTime() - t_start < duration_s:
        if sup.step(timestep_ms) == -1:
            return None
        seq, sample = source.latest_with_seq()
        if sample is None or seq == last_seq:
            continue
        now = sup.getTime()
        sample_dt = now - last_sample_t
        last_sample_t = now
        last_seq = seq

        ax_ms2 = (forward_accel_g(sample) - static_biases["ax_g"]) * GRAVITY_M_S2
        ay_ms2 = (left_accel_g(sample) - static_biases["ay_g"]) * GRAVITY_M_S2
        gz_dps = yaw_rate_dps(sample) - static_biases["gz_dps"]

        vx_chip += ax_ms2 * sample_dt
        vy_chip += ay_ms2 * sample_dt
        x_chip += vx_chip * sample_dt
        y_chip += vy_chip * sample_dt
        yaw_deg += gz_dps * sample_dt

        if now - last_print_t >= 1.0:
            last_print_t = now
            elapsed = now - t_start
            print(
                f"[sup]   t={elapsed:5.1f}s  drift=({x_chip:+.3f}, "
                f"{y_chip:+.3f}) m  yaw={yaw_deg:+6.2f} deg"
            )

    T = sup.getTime() - t_start
    if T <= 0.0:
        print("ERROR: drift calibration duration was 0; aborting", file=sys.stderr)
        return None

    # Solve for the constant residual that, integrated twice over T seconds,
    # produces the observed chip-frame drift.
    residual_ax_g = (2.0 * x_chip / (T * T)) / GRAVITY_M_S2
    residual_ay_g = (2.0 * y_chip / (T * T)) / GRAVITY_M_S2
    residual_gz_dps = yaw_deg / T

    return {
        "ax_g": residual_ax_g,
        "ay_g": residual_ay_g,
        "gz_dps": residual_gz_dps,
        "_observed_drift": (x_chip, y_chip, yaw_deg),
        "_duration": T,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use a synthetic IMU source instead of connecting via BLE.",
    )
    parser.add_argument(
        "--calibrate-drift",
        type=float,
        metavar="SECONDS",
        default=None,
        help=(
            "Run a manual drift-calibration pass. After the normal static "
            "bias window, sit perfectly still for SECONDS more (30-60 is a "
            "good range); the controller integrates freely with all "
            "drift-fighters disabled, infers the constant residual bias "
            "from the resulting position and yaw drift, saves it to "
            "drift_calibration.json next to this script, and exits. "
            "Normal runs auto-load that file."
        ),
    )
    parser.add_argument(
        "--no-residual",
        action="store_true",
        help=(
            "Skip loading drift_calibration.json even if it exists "
            "(use the per-session static bias only)."
        ),
    )
    args, _ = parser.parse_known_args()

    sup = Supervisor()
    timestep_ms = int(sup.getBasicTimeStep())
    dt = timestep_ms / 1000.0

    self_node = sup.getSelf()
    if self_node is None:
        print(
            "ERROR: supervisor.getSelf() returned None. Ensure the controller "
            "is attached to a Robot with `supervisor TRUE`.",
            file=sys.stderr,
        )
        sys.exit(1)
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")

    # Remember where we started so we can clamp drift relative to it.
    initial_pos = list(translation_field.getSFVec3f())
    x0, y0, z = initial_pos

    source: ImuSource = MockImuSource() if args.mock else BleImuSource()
    source.start()
    print(f"[sup] IMU source: {type(source).__name__}")

    # ------------------------------------------------------------------ #
    # Wait for the source to actually start producing samples before we
    # begin calibration. The BLE scan + connect can easily take 5-10 s,
    # which is well past the CALIB_S window -- so without this wait, the
    # calibration loop would see zero samples even though the Nano is
    # working perfectly.
    # ------------------------------------------------------------------ #
    SAMPLE_WAIT_TIMEOUT_S = 20.0
    print(f"[sup] waiting up to {SAMPLE_WAIT_TIMEOUT_S:.0f} s for the first sample...")
    wait_start_t = sup.getTime()
    while source.latest() is None:
        if sup.step(timestep_ms) == -1:
            source.stop()
            return
        if sup.getTime() - wait_start_t > SAMPLE_WAIT_TIMEOUT_S:
            print(
                f"ERROR: no IMU samples received within {SAMPLE_WAIT_TIMEOUT_S:.0f} s.\n"
                f"  - If you are running with --mock, this should be impossible "
                f"    (mock source returns data on every call); check the controller console.\n"
                f"  - If you are running against real hardware, check the [ble-imu] lines "
                f"    above. The most common causes are:\n"
                f"      * The Nano is not powered on, or not flashed with "
                f"        firmware/MovementDetection/.\n"
                f"      * The Nano is advertising under a different local name "
                f"        (expected {'MovementTwin'!r}).\n"
                f"      * Bluetooth is disabled on this PC.\n"
                f"      * Another process (e.g. the previous run, or "
                f"        python/programs/collisionDetector/) is still holding the BLE "
                f"        connection -- close it.",
                file=sys.stderr,
            )
            source.stop()
            return
    print(f"[sup] first sample received after {sup.getTime() - wait_start_t:.2f} s")

    # ------------------------------------------------------------------ #
    # Axis remapping. Edit these three helpers if your chip is mounted in
    # a different orientation. They define how the chip-frame sample feeds
    # the world-frame dead reckoner.
    # ------------------------------------------------------------------ #
    def forward_accel_g(s: Sample) -> float:
        return s[0]   # ax: forward (chip held flat, USB toward user)

    def left_accel_g(s: Sample) -> float:
        return s[1]   # ay: left

    def yaw_rate_dps(s: Sample) -> float:
        return s[5]   # gz: rotation about chip-up axis

    def vertical_accel_g(s: Sample) -> float:
        return s[2]   # az: where gravity sits when chip is flat

    # ------------------------------------------------------------------ #
    # Calibration: average a few seconds of stationary readings to recover
    # accel & gyro bias (including the ~1 g of gravity on the vertical axis).
    # ------------------------------------------------------------------ #
    print(
        f"[sup] calibrating... hold the chip still for {CALIB_S:.1f} s "
        f"(flat, USB toward you, components up)"
    )
    n_calib_steps = max(1, int(CALIB_S / dt))
    calib: List[Sample] = []
    for _ in range(n_calib_steps):
        if sup.step(timestep_ms) == -1:
            source.stop()
            return
        sample = source.latest()
        if sample is not None:
            calib.append(sample)

    if not calib:
        print(
            "ERROR: source stopped producing samples during calibration "
            "(it was producing them before we entered the window). This "
            "usually means the BLE link dropped mid-calibration. Re-run.",
            file=sys.stderr,
        )
        source.stop()
        return

    n = len(calib)
    bias_ax = sum(forward_accel_g(s) for s in calib) / n
    bias_ay = sum(left_accel_g(s) for s in calib) / n
    bias_az_with_gravity = sum(vertical_accel_g(s) for s in calib) / n
    bias_gx = sum(s[3] for s in calib) / n
    bias_gy = sum(s[4] for s in calib) / n
    bias_gz_dps = sum(yaw_rate_dps(s) for s in calib) / n
    print(
        f"[sup] static biases: accel=({bias_ax:+.4f}, {bias_ay:+.4f}, "
        f"{bias_az_with_gravity:+.4f}) g, gyro=({bias_gx:+.3f}, "
        f"{bias_gy:+.3f}, {bias_gz_dps:+.3f}) deg/s"
    )

    # ------------------------------------------------------------------ #
    # Optional manual drift calibration.
    # ------------------------------------------------------------------ #
    if args.calibrate_drift is not None:
        static_biases = {
            "ax_g": bias_ax,
            "ay_g": bias_ay,
            "gz_dps": bias_gz_dps,
        }
        axis_remap = (forward_accel_g, left_accel_g, yaw_rate_dps)
        residual = _measure_residual_drift(
            sup, source, static_biases, axis_remap,
            args.calibrate_drift, timestep_ms,
        )
        if residual is None:
            source.stop()
            return
        dx, dy, dyaw = residual["_observed_drift"]
        T = residual["_duration"]
        print(f"[sup] -- drift calibration results (over {T:.2f} s) --")
        print(f"[sup]   observed chip-frame drift: ({dx:+.3f}, {dy:+.3f}) m")
        print(f"[sup]   observed yaw drift:        {dyaw:+.3f} deg")
        print("[sup]   inferred residual biases (to subtract on future runs):")
        print(
            f"[sup]     accel: ({residual['ax_g']:+.5f}, "
            f"{residual['ay_g']:+.5f}) g"
        )
        print(f"[sup]     gyro_z: {residual['gz_dps']:+.4f} deg/s")
        if abs(residual["ax_g"]) > 0.05 or abs(residual["ay_g"]) > 0.05:
            print(
                "[sup] WARNING: residual accel bias is unusually large "
                "(> 0.05 g). The chip probably wasn't truly stationary, or "
                "isn't held flat. Re-run --calibrate-drift.",
                file=sys.stderr,
            )
        if abs(residual["gz_dps"]) > 5.0:
            print(
                "[sup] WARNING: residual gyro bias is unusually large "
                "(> 5 deg/s). Re-run --calibrate-drift.",
                file=sys.stderr,
            )
        _save_residual_calibration(residual, args.calibrate_drift, CALIB_S)
        print(f"[sup] saved -> {RESIDUAL_CALIBRATION_PATH}")
        print("[sup] done. Re-launch without --calibrate-drift to use it.")
        source.stop()
        return

    # ------------------------------------------------------------------ #
    # Load residual-bias calibration if it exists.
    # ------------------------------------------------------------------ #
    residual_bias_ax = 0.0
    residual_bias_ay = 0.0
    residual_bias_gz_dps = 0.0
    if args.no_residual:
        print("[sup] --no-residual set; ignoring drift_calibration.json")
    else:
        residual = _load_residual_calibration()
        if residual is not None:
            residual_bias_ax = residual["residual_bias_ax_g"]
            residual_bias_ay = residual["residual_bias_ay_g"]
            residual_bias_gz_dps = residual["residual_bias_gz_dps"]
            print(
                f"[sup] loaded residual calibration (captured "
                f"{residual['calibration_timestamp']}, "
                f"window {residual['calibration_duration_s']:.1f} s):"
            )
            print(
                f"[sup]   accel: ({residual_bias_ax:+.5f}, "
                f"{residual_bias_ay:+.5f}) g, "
                f"gyro_z: {residual_bias_gz_dps:+.4f} deg/s"
            )
        else:
            print(
                "[sup] no drift_calibration.json found -- using static "
                "bias only. Run once with --calibrate-drift 30 to measure "
                "and persist the residual."
            )

    # ------------------------------------------------------------------ #
    # Dead reckoning state.
    # ------------------------------------------------------------------ #
    yaw_rad = 0.0
    vx = 0.0
    vy = 0.0
    x = x0
    y = y0
    zupt_counter = 0
    last_print_t = 0.0
    PRINT_INTERVAL_S = 0.5

    # Sequence-aware integration: only advance the integrator when we receive
    # a fresh sample (avoids double-counting acceleration during BLE
    # stutters). Uses the actual elapsed wall-clock time between samples as
    # dt instead of the supervisor's fixed tick (which is a zero-order hold
    # assumption that overestimates motion during transients).
    last_seq, _ = source.latest_with_seq()
    last_sample_t = sup.getTime()

    # Low-pass-filtered accelerations (chip frame, m/s^2, bias-corrected).
    ax_lpf = 0.0
    ay_lpf = 0.0

    print("[sup] tracking. Drift = inevitable; ZUPT + damping keep it bounded.")

    while sup.step(timestep_ms) != -1:
        seq, sample = source.latest_with_seq()
        if sample is None:
            continue

        is_new_sample = (seq != last_seq)
        if is_new_sample:
            now = sup.getTime()
            sample_dt = now - last_sample_t
            last_sample_t = now
            last_seq = seq
        else:
            sample_dt = 0.0   # don't integrate on a duplicate sample

        # Bias-correct (static cal + persisted residual, both in chip frame).
        ax_g = forward_accel_g(sample) - bias_ax - residual_bias_ax
        ay_g = left_accel_g(sample) - bias_ay - residual_bias_ay
        gz_dps = yaw_rate_dps(sample) - bias_gz_dps - residual_bias_gz_dps

        # Convert to SI.
        ax_ms2_raw = ax_g * GRAVITY_M_S2
        ay_ms2_raw = ay_g * GRAVITY_M_S2
        yaw_rate = math.radians(gz_dps)

        # Single-pole IIR low-pass on chip-frame accel (only when we have a
        # new sample, so the time constant tracks the sample stream rather
        # than the supervisor tick).
        if is_new_sample:
            ax_lpf = ACCEL_LPF_ALPHA * ax_ms2_raw + (1.0 - ACCEL_LPF_ALPHA) * ax_lpf
            ay_lpf = ACCEL_LPF_ALPHA * ay_ms2_raw + (1.0 - ACCEL_LPF_ALPHA) * ay_lpf

        # Integrate yaw (using sample_dt; 0 on a duplicate).
        yaw_rad += yaw_rate * sample_dt * YAW_RATE_SCALE

        # Rotate horizontal accel from chip frame into world frame using the
        # current yaw estimate. This treats the chip as roughly level (no
        # pitch/roll tracking), which is fine for a flat trajectory.
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        ax_world = ax_lpf * c - ay_lpf * s
        ay_world = ax_lpf * s + ay_lpf * c

        # ZUPT: detect stationary chip and snap velocity to zero. Use the
        # smoothed accel + raw gyro magnitudes.
        accel_mag = math.sqrt(ax_lpf * ax_lpf + ay_lpf * ay_lpf)
        gyro_mag = abs(gz_dps)
        if is_new_sample:
            if (accel_mag < ZUPT_ACCEL_THRESHOLD_M_S2
                    and gyro_mag < ZUPT_GYRO_THRESHOLD_DPS):
                zupt_counter += 1
                if zupt_counter >= ZUPT_WINDOW:
                    vx = 0.0
                    vy = 0.0
            else:
                zupt_counter = 0

        # Integrate velocity (with proper per-sample dt; 0 on a duplicate).
        vx += ax_world * sample_dt
        vy += ay_world * sample_dt

        # Apply velocity damping every tick (independent of sample arrival,
        # so the system bleeds off integrated noise on a predictable
        # schedule).
        vx *= VELOCITY_DECAY
        vy *= VELOCITY_DECAY

        # Apply user-facing speed scale and clamp.
        vx_out = vx * SPEED_SCALE
        vy_out = vy * SPEED_SCALE
        speed_out = math.sqrt(vx_out * vx_out + vy_out * vy_out)
        if speed_out > MAX_SPEED_M_S:
            scale = MAX_SPEED_M_S / speed_out
            vx_out *= scale
            vy_out *= scale
            # Pull the internal velocity back too, so the clamp is sticky.
            vx *= scale
            vy *= scale

        # Integrate position using the *scaled* output velocity, advanced by
        # the supervisor tick (we want smooth visible motion every frame,
        # even when no fresh sample arrived).
        x += vx_out * dt
        y += vy_out * dt

        # Soft position clamp (keeps drift inside the visible arena).
        r = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
        if r > MAX_RADIUS_M:
            scale = MAX_RADIUS_M / r
            x = x0 + (x - x0) * scale
            y = y0 + (y - y0) * scale
            vx = 0.0
            vy = 0.0

        # Apply pose to the robot.
        translation_field.setSFVec3f([x, y, z])
        rotation_field.setSFRotation([0.0, 0.0, 1.0, yaw_rad])

        # Throttled diagnostic print.
        now = sup.getTime()
        if now - last_print_t >= PRINT_INTERVAL_S:
            last_print_t = now
            print(
                f"[sup] pose=({x:+.3f}, {y:+.3f}) m, "
                f"yaw={math.degrees(yaw_rad):+6.1f} deg, "
                f"speed={speed_out:.3f} m/s"
            )

    source.stop()


if __name__ == "__main__":
    main()
