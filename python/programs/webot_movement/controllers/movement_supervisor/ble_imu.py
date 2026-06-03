"""Thread-safe IMU source for the Webots movement supervisor.

Mirrors the structure of `ble_proximity.py` in the sibling `webot/` project,
but for the BMI270 IMU on the Nano 33 BLE Sense.

Wire format (must match `firmware/MovementDetection/src/main.cpp`):
    12 bytes per notification, little-endian: int16 ax, ay, az, gx, gy, gz
    accel scaled by 16384 (full scale +-2 g)
    gyro  scaled by 131   (full scale +-250 deg/s)
"""

from __future__ import annotations

import asyncio
import math
import struct
import threading
import time
from typing import Optional, Tuple

DEVICE_NAME = "MovementTwin"
IMU_UUID = "19B10011-E8F2-537E-4F6C-D104768A1214"

ACCEL_SCALE = 16384.0  # int16 -> g
GYRO_SCALE = 131.0     # int16 -> deg/s

# (ax, ay, az) in g; (gx, gy, gz) in deg/s. All in chip-local frame.
Sample = Tuple[float, float, float, float, float, float]


def parse_imu_packet(data: bytes) -> Optional[Sample]:
    """Decode 12-byte BLE payload into a 6-tuple of physical units."""
    if len(data) != 12:
        return None
    ax, ay, az, gx, gy, gz = struct.unpack("<6h", data)
    return (
        ax / ACCEL_SCALE,
        ay / ACCEL_SCALE,
        az / ACCEL_SCALE,
        gx / GYRO_SCALE,
        gy / GYRO_SCALE,
        gz / GYRO_SCALE,
    )


class ImuSource:
    """Interface for `BleImuSource` and `MockImuSource`.

    `latest()` returns the most recently received sample, or `None` if no
    sample has been received yet.

    `latest_with_seq()` returns `(seq, sample)` where `seq` is a
    monotonically increasing counter incremented every time a fresh sample
    arrives. Two consecutive calls that return the same `seq` mean no new
    sample arrived in between, so the consumer can avoid double-integrating
    the same reading when its tick rate is faster than the source's update
    rate (a common BLE pattern: 50 Hz source vs ~31 Hz supervisor tick).
    """

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def latest(self) -> Optional[Sample]:
        raise NotImplementedError

    def latest_with_seq(self) -> Tuple[int, Optional[Sample]]:
        raise NotImplementedError


class MockImuSource(ImuSource):
    """Deterministic IMU stream for offline development.

    Simulates a chip held flat (so az ~= +1 g) that, after a calibration
    window, performs a slow forward push every 6 s:
        t in [0, 3) s         : at rest
        t in [3, 3.5) s       : ax = +0.03 g  (accelerate forward)
        t in [3.5, 4) s       : ax = -0.03 g  (decelerate, stop)
        t in [4, 6) s         : at rest (new position)
        repeat with period 6 s

    With perfect dead reckoning that displaces the virtual robot by
        d = a * dt^2 / 2 + a * dt * dt / 2 = 0.03 * 9.81 * 0.5^2 ~= 0.074 m
    forward per cycle, which is comfortably visible inside a 3 m arena.
    """

    PERIOD_S = 6.0
    PULSE_G = 0.03
    PULSE_DURATION_S = 0.5
    # Pretend the mock arrives at 50 Hz (same nominal rate as the firmware),
    # so seq numbers in `latest_with_seq` advance the same way as for BLE.
    SAMPLE_PERIOD_S = 1.0 / 50.0

    def __init__(self) -> None:
        self._t0 = time.time()

    def start(self) -> None:
        self._t0 = time.time()

    def stop(self) -> None:
        pass

    def _ax_at(self, t: float) -> float:
        # First 3 s are the calibration window: stay perfectly still.
        if t < 3.0:
            return 0.0
        phase = (t - 3.0) % self.PERIOD_S
        if phase < self.PULSE_DURATION_S:
            return +self.PULSE_G
        if phase < 2 * self.PULSE_DURATION_S:
            return -self.PULSE_G
        return 0.0

    def latest(self) -> Optional[Sample]:
        t = time.time() - self._t0
        return (self._ax_at(t), 0.0, 1.0, 0.0, 0.0, 0.0)

    def latest_with_seq(self) -> Tuple[int, Optional[Sample]]:
        t = time.time() - self._t0
        seq = int(t / self.SAMPLE_PERIOD_S)
        return (seq, (self._ax_at(t), 0.0, 1.0, 0.0, 0.0, 0.0))


class BleImuSource(ImuSource):
    """Connects to the MovementTwin Nano in a background thread."""

    def __init__(
        self,
        device_name: str = DEVICE_NAME,
        scan_timeout_s: float = 5.0,
    ) -> None:
        self._device_name = device_name
        self._scan_timeout = scan_timeout_s
        self._latest: Optional[Sample] = None
        self._seq: int = 0
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="ble-imu"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def latest(self) -> Optional[Sample]:
        with self._lock:
            return self._latest

    def latest_with_seq(self) -> Tuple[int, Optional[Sample]]:
        with self._lock:
            return (self._seq, self._latest)

    def _handle_notification(self, _sender, data: bytearray) -> None:
        sample = parse_imu_packet(bytes(data))
        if sample is None:
            return
        with self._lock:
            self._latest = sample
            self._seq += 1

    def _run(self) -> None:
        try:
            asyncio.run(self._async_main())
        except Exception as exc:  # controller must keep running on BLE errors
            print(f"[ble-imu] thread terminated: {exc!r}")

    async def _async_main(self) -> None:
        # Import inside the thread so a missing bleak install only breaks the
        # BLE path, not `--mock` runs.
        from bleak import BleakClient, BleakScanner

        print(f"[ble-imu] scanning for {self._device_name!r}...")
        devices = await BleakScanner.discover(timeout=self._scan_timeout)
        target = next((d for d in devices if d.name == self._device_name), None)
        if target is None:
            print(f"[ble-imu] {self._device_name!r} not found; staying idle.")
            return

        print(f"[ble-imu] connecting to {target.address}...")
        async with BleakClient(target.address) as client:
            await client.start_notify(IMU_UUID, self._handle_notification)
            print("[ble-imu] streaming IMU at ~50 Hz.")
            while not self._stop_evt.is_set():
                await asyncio.sleep(0.05)
            await client.stop_notify(IMU_UUID)
