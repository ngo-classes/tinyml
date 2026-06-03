"""Webots Supervisor controller for the Nano + APDS9960 digital twin.

Reads the live APDS9960 proximity stream (0..255, where 0 = very close,
255 = nothing detected) from the physical Arduino Nano 33 BLE over Bluetooth
LE, scales it to a simulated distance, and moves the pre-declared
`DEF PROX_OBSTACLE` node in `worlds/open_field.wbt` to that point in front
of the NanoAPDS9960 robot.

When the physical sensor reports "nothing" (reading >= TRIGGER_THRESHOLD) the
obstacle is parked underground at z = -10 to hide it.

Run with `--mock` (the default in `open_field.wbt`) for a deterministic sweep
that does not require the physical hardware.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Tuple

from controller import Supervisor  # type: ignore[import-not-found]

from ble_proximity import BleProximitySource, MockProximitySource, ProximitySource

# --- Mapping configuration ------------------------------------------------- #
# APDS9960: 0 (very close) -> 255 (nothing). Anything at/above the threshold
# means "no obstacle visible to the physical sensor", so we hide the prop.
TRIGGER_THRESHOLD = 200

# Scaled distance range in meters between the robot's front face and the
# obstacle center.
MIN_DISTANCE_M = 0.05
MAX_DISTANCE_M = 1.50

# Must match the DEF name used in `worlds/open_field.wbt`.
OBSTACLE_DEF = "PROX_OBSTACLE"

# Same height/radius as the cylinder declared in the world file. Used only to
# compute the obstacle's Z so it sits on the floor.
OBSTACLE_HEIGHT_M = 0.10

# Where the obstacle lives when "nothing detected". Well below the floor.
HIDDEN_POSITION = [0.0, 0.0, -10.0]


def scale_proximity_to_meters(p: int) -> float:
    """Linearly map APDS9960 reading [0..255] -> [MIN..MAX] meters."""
    p = max(0, min(255, int(p)))
    frac = p / 255.0
    return MIN_DISTANCE_M + frac * (MAX_DISTANCE_M - MIN_DISTANCE_M)


def axis_angle_rotate(
    vec: Tuple[float, float, float],
    axis: Tuple[float, float, float],
    angle: float,
) -> Tuple[float, float, float]:
    """Rotate `vec` by `angle` rad around `axis` using Rodrigues' formula."""
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / norm, ay / norm, az / norm
    c, s = math.cos(angle), math.sin(angle)
    vx, vy, vz = vec
    dot = ax * vx + ay * vy + az * vz
    rx = vx * c + (ay * vz - az * vy) * s + ax * dot * (1.0 - c)
    ry = vy * c + (az * vx - ax * vz) * s + ay * dot * (1.0 - c)
    rz = vz * c + (ax * vy - ay * vx) * s + az * dot * (1.0 - c)
    return (rx, ry, rz)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use a synthetic proximity source instead of connecting via BLE.",
    )
    args, _unknown = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()

    sup = Supervisor()
    timestep = int(sup.getBasicTimeStep())

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

    obstacle = sup.getFromDef(OBSTACLE_DEF)
    if obstacle is None:
        print(
            f"ERROR: DEF {OBSTACLE_DEF} not found in world. Make sure the "
            f"world file pre-declares `DEF {OBSTACLE_DEF} Solid {{ ... }}`.",
            file=sys.stderr,
        )
        sys.exit(1)
    obstacle_translation = obstacle.getField("translation")

    source: ProximitySource = (
        MockProximitySource() if args.mock else BleProximitySource()
    )
    source.start()
    print(f"[sup] proximity source: {type(source).__name__}")
    print(f"[sup] tracking obstacle DEF {OBSTACLE_DEF}")

    # Print a state change only when visibility flips, so we don't spam the
    # console every 32 ms.
    visible: bool = False

    try:
        while sup.step(timestep) != -1:
            reading = source.latest()
            if reading is None:
                continue

            if reading >= TRIGGER_THRESHOLD:
                if visible:
                    obstacle_translation.setSFVec3f(HIDDEN_POSITION)
                    visible = False
                    print(f"[sup] obstacle hidden (proximity={reading})")
                continue

            d = scale_proximity_to_meters(reading)
            x0, y0, _z0 = translation_field.getSFVec3f()
            ax, ay, az, ang = rotation_field.getSFRotation()
            # The PROTO's local +X axis is "forward" (where the APDS9960 looks).
            fx, fy, _fz = axis_angle_rotate((d, 0.0, 0.0), (ax, ay, az), ang)
            target = [x0 + fx, y0 + fy, OBSTACLE_HEIGHT_M / 2.0]
            obstacle_translation.setSFVec3f(target)
            if not visible:
                visible = True
                print(
                    f"[sup] obstacle visible at "
                    f"({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) "
                    f"d={d:.2f}m (proximity={reading})"
                )
    finally:
        source.stop()


if __name__ == "__main__":
    main()
