# Webots digital twin: Nano 33 BLE + APDS9960

A minimal [Webots R2025a](https://cyberbotics.com/) project that mirrors the
physical Arduino Nano 33 BLE + APDS9960 rig defined in
`firmware/CollisionDetection/`. A custom `NanoAPDS9960` robot PROTO sits in
the middle of an open arena. A Supervisor controller listens to the live
proximity stream coming over Bluetooth LE from the physical Nano and, every
time the sensor reports an object, spawns (or moves) a red cylinder in front
of the simulated rig at a scaled distance.

```
firmware/CollisionDetection (Nano 33 BLE)
        |
        |  BLE notify: APDS9960 proximity byte (0..255)
        v
python/programs/webot/controllers/twin_supervisor
        |  scale + place
        v
Webots world: red cylinder appears in front of NanoAPDS9960
```

## Layout

```
python/programs/webot/
├── README.md                       <- you are here
├── worlds/
│   └── open_field.wbt              <- ENU arena + NANO instance
├── protos/
│   └── NanoAPDS9960.proto          <- custom robot PROTO (Nano + APDS9960)
└── controllers/
    └── twin_supervisor/
        ├── twin_supervisor.py      <- spawns / moves the obstacle
        ├── ble_proximity.py        <- threaded BLE source + mock source
        └── runtime.ini             <- points Webots at system python
```

## Prerequisites

- Webots R2025a installed.
- Python 3.9+ on `PATH` (or edit `controllers/twin_supervisor/runtime.ini`
  to point at the interpreter you want Webots to use).
- `pip install bleak` in that same interpreter (only needed for real BLE
  runs; the default `--mock` mode does not need it).
- The physical Nano 33 BLE flashed with `firmware/CollisionDetection/` and
  advertising under the BLE local name `PhysicalTwin` (UUIDs are shared with
  `python/programs/collisionDetector/collisionDetector.py`).

## Running

1. Launch Webots and open `python/programs/webot/worlds/open_field.wbt`.
2. Press the play button. The world ships with `controllerArgs ["--mock"]`,
   so the obstacle should immediately start sweeping in and out in front of
   the NanoAPDS9960 rig without any hardware connected. This confirms the
   Webots + controller side is wired correctly.
3. To switch to real hardware, edit the `NANO` node in `open_field.wbt` and
   remove `--mock` from `controllerArgs`, e.g.:

   ```vrml
   DEF NANO NanoAPDS9960 {
     ...
     controllerArgs []
   }
   ```

   Then power the Nano, wait until it advertises as `PhysicalTwin`, and
   restart the simulation. The supervisor will scan for ~5 s, connect, and
   start consuming proximity notifications.

## Mapping APDS9960 -> world distance

The APDS9960 returns a single unsigned byte each notification, where `0`
means "object touching the sensor" and `255` means "nothing detected". The
supervisor maps that byte linearly to a distance in meters:

| reading | meaning              | sim distance        |
| ------- | -------------------- | ------------------- |
| 0       | object on the sensor | `MIN_DISTANCE_M`    |
| 255     | nothing detected     | `MAX_DISTANCE_M`    |
| ≥ 200   | treated as "clear"   | obstacle is removed |

`MIN_DISTANCE_M`, `MAX_DISTANCE_M`, and `TRIGGER_THRESHOLD` live at the top
of `controllers/twin_supervisor/twin_supervisor.py`. Tune them to taste.

## Frame conventions

- World: ENU (`+X` east, `+Y` north, `+Z` up).
- The `NanoAPDS9960` PROTO is built so that its local `+X` axis is the
  direction the APDS9960 looks. The supervisor reads the robot's pose every
  step and places the obstacle at `robot_pos + R(robot_rotation) * (d, 0, 0)`
  via Rodrigues' formula, so the obstacle stays in front of the rig even if
  you later rotate or drive the robot.

## Extending

- **Drive the robot**: swap the `twin_supervisor` controller for a separate
  Robot controller that consumes wheel commands, or add wheels + `HingeJoint`
  nodes to `NanoAPDS9960.proto` and let the supervisor pilot it.
- **Echo the LED**: the PROTO already exposes a Webots `LED` node named
  `rgb_led`. You can drive it from the supervisor whenever you would send
  the `"flash"` instruction to the physical Nano, keeping the digital and
  physical twins visually in sync.
- **Multiple obstacles**: change the obstacle bookkeeping in
  `twin_supervisor.py` from a single handle to a list, keyed by e.g. a
  rolling window of recent proximity events.
