# Webots digital twin: Nano 33 BLE IMU -> virtual MobileNano pose

A [Webots R2025a](https://cyberbotics.com/) project that mirrors the
*motion* of an Arduino Nano 33 BLE Sense in a virtual robot. The physical
chip streams its on-board BMI270 IMU (accelerometer + gyroscope) over BLE
at 50 Hz, and a Webots Supervisor controller performs honest dead reckoning
to recover a 2D pose, then writes that pose into the `MobileNano` robot's
`translation` and `rotation` fields every tick.

```
firmware/MovementDetection (Nano 33 BLE Sense, BMI270 IMU)
        |
        |  BLE notify: 12 bytes / 50 Hz
        |  int16 ax, ay, az, gx, gy, gz
        v
python/programs/webot_movement/controllers/movement_supervisor
        |  calibration -> bias-correct -> integrate -> ZUPT -> clamp
        v
Webots world: MobileNano teleports each tick to the dead-reckoned pose
```

## Layout

```
python/programs/webot_movement/
├── README.md                       <- you are here
├── worlds/
│   └── open_field.wbt              <- same template as ../webot/, MobileNano at origin
├── protos/
│   └── MobileNano.proto            <- small chassis + vertical chip at front
└── controllers/
    └── movement_supervisor/
        ├── movement_supervisor.py  <- dead reckoning + Webots pose update
        ├── ble_imu.py              <- threaded BLE source + deterministic mock
        └── runtime.ini             <- points Webots at system python
```

## Prerequisites

- Webots R2025a installed.
- **macOS Bluetooth:** If the controller prints `[ble-imu] scanning...` and Webots
  reports *"The process crashed some time after starting successfully"*, check
  **System Settings → Privacy & Security → Bluetooth** and allow both **Webots**
  and **Python** (the conda interpreter in `runtime.ini`). A native SIGABRT
  during scan means macOS blocked Bluetooth for the process that launched the
  controller, not a Python exception.
- **`MovementTwin` not found:** Serial text `BLE advertising as MovementTwin` only
  confirms the firmware called `BLE.advertise()` — your Mac still has to *hear*
  the radio. If another program is already connected, advertising stops; power-cycle
  the Nano and quit other BLE tools (`collisionDetector`, nRF Connect, etc.).
  The supervisor matches the name in **advertisement** data (and the IMU service
  UUID), not the Serial log.
- Python 3.9+ on `PATH` (or edit `controllers/movement_supervisor/runtime.ini`
  to point at the interpreter you want Webots to use).
- `pip install bleak` in that same interpreter (only needed for real BLE
  runs; the default `--mock` mode does not need it).
- An Arduino Nano 33 BLE **Sense** flashed with
  `firmware/MovementDetection/`, advertising under the BLE local name
  `MovementTwin`.

## Running

1. Open `python/programs/webot_movement/worlds/open_field.wbt` in Webots.
2. Press play. The world ships with `controllerArgs ["--mock"]`, so the
   supervisor runs a deterministic synthetic IMU stream (3 s of stillness
   for the calibration window, then a slow forward push every 6 s). You
   should immediately see the small `MobileNano` chassis tick forward in
   the arena and stop, repeatedly, without any hardware connected.
3. To switch to the physical Nano, edit the `NANO` node in `open_field.wbt`
   and remove `--mock`:

   ```vrml
   DEF NANO MobileNano {
     ...
     controllerArgs []
   }
   ```

   Place the chip flat on a stable surface (USB toward you, components up),
   power it, wait for it to advertise as `MovementTwin`, then press play.
   The supervisor will sit through a 2 s calibration window during which
   **you must not touch the chip** -- it's measuring bias. Then you can
   slowly slide the chip around and watch the virtual robot follow.

## About dead reckoning

This is honest IMU-only dead reckoning, which drifts. There is no
magnetometer aiding, no external position reference, no SLAM. The pipeline
is:

```
raw IMU
  |
  v  subtract bias (measured during a 2 s stationary calibration)
  |
  v  integrate gyro_z -> yaw
  |
  v  rotate horizontal accel from chip frame into world frame using yaw
  |
  v  integrate accel  -> velocity     (+ ZUPT, + damping, + speed clamp)
  |
  v  integrate velocity -> position   (+ soft position clamp inside arena)
  |
  v  write translation + rotation into the MobileNano Robot node
```

Three knobs at the top of `movement_supervisor.py` keep this watchable for
tens of seconds of slow, steady motion:

| Knob | Default | What it does |
| --- | --- | --- |
| `CALIB_S` | 2.0 s | Window of stationary samples averaged to estimate the bias of every IMU axis (including the ~1 g of gravity on `az`). |
| `ACCEL_LPF_ALPHA` | 0.3 | Single-pole IIR on bias-corrected accel before integration. Smaller = more smoothing, less drift from noise, more lag. |
| `ZUPT_ACCEL_THRESHOLD_M_S2`, `ZUPT_GYRO_THRESHOLD_DPS`, `ZUPT_WINDOW` | 0.15 m/s^2, 2.0 deg/s, 10 samples | If the corrected accel AND gyro both stay below their thresholds for this many consecutive samples, force velocity back to zero. This is the single biggest thing that keeps the demo from drifting away on its own. |
| `VELOCITY_DECAY`, `MAX_SPEED_M_S`, `MAX_RADIUS_M` | 0.99, 0.5 m/s, 1.4 m | Velocity bleeds toward zero each tick, speed is capped, and if the position drifts past `MAX_RADIUS_M` from the start point it is softly pulled back in. |
| `SPEED_SCALE`, `YAW_RATE_SCALE` | 0.5, 0.5 | Multiplied into the *output* velocity / yaw rate. Slows the visible motion proportionally without changing drift per unit of real motion. |

**Realistic expectations:** for very slow, deliberate flat motion over a
~30 cm path, the virtual robot's final pose typically lands within ~10 cm
of where you put the chip. For faster motion, longer durations, or any
tilt, the drift will be visible. This is not a SLAM rig, and the
limitations are fundamental to 6-DoF inertial navigation.

### Manual drift calibration

The 2-second static window can only average so much, so a small residual
bias usually survives it. You can measure that residual and persist it to
disk in a one-shot calibration run.

In `open_field.wbt`, temporarily set the controller args to include
`--calibrate-drift`:

```vrml
DEF NANO MobileNano {
  ...
  controllerArgs [
    "--calibrate-drift", "30"
  ]
}
```

Press play, sit through the normal startup, then hold the chip
**absolutely still** (set it on a stable surface; don't touch it or the
table) for the requested 30 seconds. The supervisor will print the
observed chip-frame drift each second, and at the end save the inferred
residual to `controllers/movement_supervisor/drift_calibration.json` and
exit. Example output:

```
[sup] DRIFT CALIBRATION: hold the chip perfectly still for 30.0 s.
[sup]   t=  1.0s  drift=(+0.001, +0.000) m  yaw= +0.12 deg
[sup]   t= 10.0s  drift=(+0.043, -0.018) m  yaw= +1.40 deg
[sup]   t= 30.0s  drift=(+0.385, -0.160) m  yaw= +4.20 deg
[sup] -- drift calibration results (over 30.00 s) --
[sup]   observed chip-frame drift: (+0.385, -0.160) m
[sup]   observed yaw drift:        +4.200 deg
[sup]   inferred residual biases (to subtract on future runs):
[sup]     accel: (+0.00087, -0.00036) g
[sup]     gyro_z: +0.1400 deg/s
[sup] saved -> .../drift_calibration.json
```

Now remove `--calibrate-drift` from the controller args. The next normal
run will auto-load `drift_calibration.json` and subtract the residual in
addition to the per-session static bias:

```
[sup] loaded residual calibration (captured 2026-06-03T10:00:00, window 30.0 s):
[sup]   accel: (+0.00087, -0.00036) g, gyro_z: +0.1400 deg/s
```

Pass `--no-residual` to temporarily ignore the saved file without deleting
it. Re-calibrate when the chip's orientation or temperature changes
substantially -- IMU residuals are temperature-sensitive, so a calibration
done warm doesn't necessarily fit a cold chip and vice-versa.

`drift_calibration.json` is per-machine state and is `.gitignore`d.

## Physical chip orientation

The controller assumes the chip is held **flat**, with the USB connector
pointed toward the user and the components facing up. In that orientation
the chip-local axes line up with the user's frame:

| Chip axis | User frame | Used for |
| --- | --- | --- |
| `+chip-X` | forward (away from user) | dead-reckoned forward acceleration |
| `+chip-Y` | left | dead-reckoned lateral acceleration |
| `+chip-Z` | up (gravity ~+1 g) | calibrated away (gravity reference) |
| `gyro_z`  | yaw rate about up | dead-reckoned heading |

If you'd rather mount the chip vertically (as the `MobileNano` PROTO depicts
visually), edit the four helpers at the top of `main` in
`movement_supervisor.py`:

```python
def forward_accel_g(s):  return s[0]   # which sample axis is forward?
def left_accel_g(s):     return s[1]   # which sample axis is left?
def yaw_rate_dps(s):     return s[5]   # which gyro axis is yaw?
def vertical_accel_g(s): return s[2]   # which sample axis carries gravity?
```

For example, if the chip is mounted vertically with PCB face forward (chip
long-axis up, chip-Z normal forward), you'd typically use:

```python
def forward_accel_g(s):  return s[2]   # chip-Z is forward
def left_accel_g(s):     return s[1]   # chip-Y is left
def yaw_rate_dps(s):     return s[3]   # chip-X (up) is yaw
def vertical_accel_g(s): return s[0]   # chip-X is up
```

## Wire protocol

The firmware (`firmware/MovementDetection/src/main.cpp`) sends a 12-byte
packet per notification, little-endian:

```
struct ImuPacket {
  int16_t ax, ay, az;   // scaled by 16384  ->  g     (full scale +-2 g)
  int16_t gx, gy, gz;   // scaled by 131    ->  deg/s (full scale +-250 deg/s)
};
```

12 bytes fits inside the default BLE ATT MTU (23 bytes), so this works
without negotiating a larger MTU. The supervisor decodes with
`struct.unpack("<6h", data)` and reapplies the scale factors in
`ble_imu.py::parse_imu_packet`.
