"""Webots Supervisor controller for the IMU digital twin.

Reads the live IMU stream from the Nano 33 BLE Sense, integrates horizontal
acceleration into a 2D pose, and writes `translation` on the MobileNano robot.

Mounting assumption (vertical, PCB sensor facing forward, sliding on a flat
surface):

    chip +Z  ->  robot +X  (forward / backward)
    chip +Y  ->  robot +Y  (left / right)
    chip +X  ->  up (gravity; calibrated away, not integrated)

Heading is fixed — no gyro / yaw integration. Bias handling lives in
`calibration.py` (static window + optional persisted residual file).
"""

from __future__ import annotations

import argparse
import math
import sys

from controller import Supervisor  # type: ignore[import-not-found]

from ble_imu import BleImuSource, MockImuSource, ImuSource, Sample
from calibration import (
    GRAVITY_M_S2,
    ResidualBiases,
    StaticBiases,
    correct_forward_g,
    correct_left_g,
    load_residual,
    measure_residual_drift,
    run_static_calibration,
    save_residual,
    RESIDUAL_CALIBRATION_PATH,
)


def _step(sup: Supervisor, timestep_ms: int, source: ImuSource) -> int:
    """Run one Webots tick and let asyncio-backed BLE sources make progress."""
    result = sup.step(timestep_ms)
    source.pump()
    return result


ZUPT_ACCEL_THRESHOLD_M_S2 = 0.15
ZUPT_WINDOW = 10

VELOCITY_DECAY = 0.99
SPEED_SCALE = 0.5
MAX_SPEED_M_S = 0.5
MAX_RADIUS_M = 1.4
ACCEL_LPF_ALPHA = 0.3


def forward_accel_g(s: Sample) -> float:
    return s[2]   # chip +Z -> robot +X


def left_accel_g(s: Sample) -> float:
    return s[1]   # chip +Y -> robot +Y


def gravity_accel_g(s: Sample) -> float:
    return s[0]   # chip +X carries ~1 g when upright; not integrated


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
            "After static calibration, sit still for SECONDS, infer residual "
            "biases, save drift_calibration.json, and exit."
        ),
    )
    parser.add_argument(
        "--no-residual",
        action="store_true",
        help="Skip loading drift_calibration.json.",
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

    initial_pos = list(translation_field.getSFVec3f())
    x0, y0, z = initial_pos
    fixed_rotation = list(rotation_field.getSFRotation())

    source: ImuSource = MockImuSource() if args.mock else BleImuSource()
    source.start()
    print(f"[sup] IMU source: {type(source).__name__}")

    SAMPLE_WAIT_TIMEOUT_S = 20.0
    print(f"[sup] waiting up to {SAMPLE_WAIT_TIMEOUT_S:.0f} s for the first sample...")
    wait_start_t = sup.getTime()
    while source.latest() is None:
        if _step(sup, timestep_ms, source) == -1:
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

    static = run_static_calibration(
        sup,
        source,
        _step,
        timestep_ms,
        forward_accel_g,
        left_accel_g,
        gravity_accel_g,
    )
    if static is None:
        source.stop()
        return

    if args.calibrate_drift is not None:
        result = measure_residual_drift(
            sup,
            source,
            _step,
            static,
            forward_accel_g,
            left_accel_g,
            args.calibrate_drift,
            timestep_ms,
        )
        if result is None:
            source.stop()
            return
        residual_forward_g, residual_left_g, dx, dy = result
        print(f"[cal] drift results: observed=({dx:+.3f}, {dy:+.3f}) m")
        print(
            f"[cal] residual biases: forward={residual_forward_g:+.5f} g, "
            f"left={residual_left_g:+.5f} g"
        )
        if abs(residual_forward_g) > 0.05 or abs(residual_left_g) > 0.05:
            print(
                "[cal] WARNING: residual bias > 0.05 g — chip may not have been still.",
                file=sys.stderr,
            )
        save_residual(residual_forward_g, residual_left_g, args.calibrate_drift)
        print(f"[cal] saved -> {RESIDUAL_CALIBRATION_PATH}")
        source.stop()
        return

    residual = ResidualBiases(0.0, 0.0)
    if args.no_residual:
        print("[sup] --no-residual: skipping drift_calibration.json")
    else:
        loaded = load_residual()
        if loaded is not None:
            residual = loaded
            print(
                f"[sup] loaded residual ({loaded.calibration_timestamp}, "
                f"{loaded.calibration_duration_s:.1f} s): "
                f"forward={loaded.forward_g:+.5f} g, left={loaded.left_g:+.5f} g"
            )
        else:
            print("[sup] no drift_calibration.json — static bias only")

    v_forward = 0.0
    v_left = 0.0
    x = x0
    y = y0
    zupt_counter = 0
    last_print_t = 0.0
    PRINT_INTERVAL_S = 0.5

    last_seq, _ = source.latest_with_seq()
    last_sample_t = sup.getTime()
    forward_lpf = 0.0
    left_lpf = 0.0

    print("[sup] tracking (planar, fixed heading).")

    while _step(sup, timestep_ms, source) != -1:
        seq, sample = source.latest_with_seq()
        if sample is None:
            continue

        is_new_sample = seq != last_seq
        if is_new_sample:
            now = sup.getTime()
            sample_dt = now - last_sample_t
            last_sample_t = now
            last_seq = seq
        else:
            sample_dt = 0.0

        forward_ms2 = correct_forward_g(sample, static, residual, forward_accel_g) * GRAVITY_M_S2
        left_ms2 = correct_left_g(sample, static, residual, left_accel_g) * GRAVITY_M_S2

        if is_new_sample:
            forward_lpf = (
                ACCEL_LPF_ALPHA * forward_ms2
                + (1.0 - ACCEL_LPF_ALPHA) * forward_lpf
            )
            left_lpf = (
                ACCEL_LPF_ALPHA * left_ms2
                + (1.0 - ACCEL_LPF_ALPHA) * left_lpf
            )

        accel_mag = math.sqrt(forward_lpf * forward_lpf + left_lpf * left_lpf)
        if is_new_sample:
            if accel_mag < ZUPT_ACCEL_THRESHOLD_M_S2:
                zupt_counter += 1
                if zupt_counter >= ZUPT_WINDOW:
                    v_forward = 0.0
                    v_left = 0.0
            else:
                zupt_counter = 0

        v_forward += forward_lpf * sample_dt
        v_left += left_lpf * sample_dt

        v_forward *= VELOCITY_DECAY
        v_left *= VELOCITY_DECAY

        v_forward_out = v_forward * SPEED_SCALE
        v_left_out = v_left * SPEED_SCALE
        speed_out = math.sqrt(v_forward_out * v_forward_out + v_left_out * v_left_out)
        if speed_out > MAX_SPEED_M_S:
            scale = MAX_SPEED_M_S / speed_out
            v_forward_out *= scale
            v_left_out *= scale
            v_forward *= scale
            v_left *= scale

        x += v_forward_out * dt
        y += v_left_out * dt

        r = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
        if r > MAX_RADIUS_M:
            scale = MAX_RADIUS_M / r
            x = x0 + (x - x0) * scale
            y = y0 + (y - y0) * scale
            v_forward = 0.0
            v_left = 0.0

        translation_field.setSFVec3f([x, y, z])
        rotation_field.setSFRotation(fixed_rotation)

        now = sup.getTime()
        if now - last_print_t >= PRINT_INTERVAL_S:
            last_print_t = now
            print(
                f"[sup] pose=({x - x0:+.3f}, {y - y0:+.3f}) m, "
                f"speed={speed_out:.3f} m/s"
            )

    source.stop()


if __name__ == "__main__":
    main()
