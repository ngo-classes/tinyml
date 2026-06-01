"""Webots Supervisor controller for the Nano + APDS9960 digital twin.

Reads the live APDS9960 proximity stream (0..255, where 0 = very close,
255 = nothing detected) from the physical Arduino Nano 33 BLE over Bluetooth
LE, scales it to a simulated distance, and spawns / moves a red cylinder
obstacle in front of the NanoAPDS9960 robot at that distance.

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
# means "no obstacle visible to the physical sensor", so we remove the prop.
TRIGGER_THRESHOLD = 200

# Scaled distance range in meters between the robot's front face and the
# obstacle center.
MIN_DISTANCE_M = 0.05
MAX_DISTANCE_M = 1.50

# Obstacle geometry.
OBSTACLE_DEF = "PROX_OBSTACLE"
OBSTACLE_HEIGHT_M = 0.10
OBSTACLE_RADIUS_M = 0.03

def _build_obstacle_node(x: float, y: float, z: float) -> str:
    """Render a VRML `Solid { ... }` string for the proximity obstacle.

    Built with an f-string (where `{{` / `}}` produce literal braces) so we
    don't have to juggle a separate `.format()` template against VRML's own
    use of curly braces.

    NOTE on orientation: Webots' `Cylinder` primitive has its axis along the
    local Y axis. The world is ENU (Z = up), so to stand the cylinder upright
    we wrap the `Shape` in a `Pose` rotated 90 deg about X (Y -> Z).
    """
    return (
        f"DEF {OBSTACLE_DEF} Solid {{\n"
        f"  translation {x} {y} {z}\n"
        f'  name "prox_obstacle"\n'
        f"  children [\n"
        f"    Pose {{\n"
        f"      rotation 1 0 0 1.5707963\n"
        f"      children [\n"
        f"        Shape {{\n"
        f"          appearance PBRAppearance {{\n"
        f"            baseColor 0.9 0.1 0.1\n"
        f"            roughness 0.5\n"
        f"            metalness 0.0\n"
        f"          }}\n"
        f"          geometry Cylinder {{\n"
        f"            height {OBSTACLE_HEIGHT_M}\n"
        f"            radius {OBSTACLE_RADIUS_M}\n"
        f"          }}\n"
        f"        }}\n"
        f"      ]\n"
        f"    }}\n"
        f"  ]\n"
        f"}}\n"
    )


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
    # Webots may pass its own arguments; ignore anything we don't recognize.
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
    root_children = sup.getRoot().getField("children")

    source: ProximitySource = (
        MockProximitySource() if args.mock else BleProximitySource()
    )
    source.start()
    print(f"[sup] proximity source: {type(source).__name__}")

    obstacle_node = None

    try:
        while sup.step(timestep) != -1:
            reading = source.latest()
            if reading is None:
                continue

            if reading >= TRIGGER_THRESHOLD:
                if obstacle_node is not None:
                    obstacle_node.remove()
                    obstacle_node = None
                continue

            d = scale_proximity_to_meters(reading)
            x0, y0, _z0 = translation_field.getSFVec3f()
            ax, ay, az, ang = rotation_field.getSFRotation()
            # The PROTO's local +X axis is "forward" (where the APDS9960 looks).
            fx, fy, _fz = axis_angle_rotate((d, 0.0, 0.0), (ax, ay, az), ang)
            target = [x0 + fx, y0 + fy, OBSTACLE_HEIGHT_M / 2.0]

            if obstacle_node is None:
                node_string = _build_obstacle_node(target[0], target[1], target[2])
                before = root_children.getCount()
                root_children.importMFNodeFromString(-1, node_string)
                after = root_children.getCount()
                # `getMFNode(-1)` is more reliable than `getFromDef` for nodes
                # inserted at runtime: it returns the most recently appended
                # child directly, without depending on the DEF dictionary.
                obstacle_node = root_children.getMFNode(-1)
                if obstacle_node is None or after == before:
                    print(
                        f"[sup] WARN: import did not add a node "
                        f"(children {before} -> {after}). "
                        f"node_string was:\n{node_string}",
                        file=sys.stderr,
                    )
                else:
                    print(
                        f"[sup] spawned obstacle at "
                        f"({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) "
                        f"for proximity reading {reading}"
                    )
            else:
                obstacle_node.getField("translation").setSFVec3f(target)
    finally:
        source.stop()


if __name__ == "__main__":
    main()
