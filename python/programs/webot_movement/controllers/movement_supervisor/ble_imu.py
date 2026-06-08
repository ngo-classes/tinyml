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
import time
from typing import Optional, Tuple

DEVICE_NAME = "MovementTwin"
# Must match firmware/MovementDetection/src/main.cpp (service + notify char).
IMU_SERVICE_UUID = "19B10010-E8F2-537E-4F6C-D104768A1214"
IMU_CHAR_UUID = "19B10011-E8F2-537E-4F6C-D104768A1214"

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

    def pump(self) -> None:
        """Advance an asyncio-backed source. No-op for synchronous sources."""

    def stop(self) -> None:
        raise NotImplementedError

    def latest(self) -> Optional[Sample]:
        raise NotImplementedError

    def latest_with_seq(self) -> Tuple[int, Optional[Sample]]:
        raise NotImplementedError


class MockImuSource(ImuSource):
    """Deterministic IMU stream for offline development.

    Simulates the vertical mount (gravity on chip +X, motion on chip +Z):
    after a calibration window, a slow forward push every 6 s on chip +Z.
    """

    PERIOD_S = 6.0
    PULSE_G = 0.03
    PULSE_DURATION_S = 0.5
    SAMPLE_PERIOD_S = 1.0 / 50.0
    GRAVITY_G = 1.0

    def __init__(self) -> None:
        self._t0 = time.time()

    def start(self) -> None:
        self._t0 = time.time()

    def stop(self) -> None:
        pass

    def _forward_az_at(self, t: float) -> float:
        if t < 3.0:
            return 0.0
        phase = (t - 3.0) % self.PERIOD_S
        if phase < self.PULSE_DURATION_S:
            return +self.PULSE_G
        if phase < 2 * self.PULSE_DURATION_S:
            return -self.PULSE_G
        return 0.0

    def _sample_at(self, t: float) -> Sample:
        # (ax, ay, az, gx, gy, gz): upright, PCB forward, gravity on +X.
        return (
            self.GRAVITY_G,
            0.0,
            self._forward_az_at(t),
            0.0,
            0.0,
            0.0,
        )

    def latest(self) -> Optional[Sample]:
        t = time.time() - self._t0
        return self._sample_at(t)

    def latest_with_seq(self) -> Tuple[int, Optional[Sample]]:
        t = time.time() - self._t0
        seq = int(t / self.SAMPLE_PERIOD_S)
        return (seq, self._sample_at(t))


class BleImuSource(ImuSource):
    """Connects to the MovementTwin Nano via asyncio on the main thread.

    Bleak/CoreBluetooth on macOS is unreliable when BLE runs in a background
    thread while Webots calls into libController on the main thread (the
    controller process can SIGABRT mid-scan). The supervisor therefore calls
    `pump()` once per `supervisor.step()` so asyncio and Webots share the
    same thread.
    """

    def __init__(
        self,
        device_name: str = DEVICE_NAME,
        scan_timeout_s: float = 15.0,
    ) -> None:
        self._device_name = device_name
        self._scan_timeout = scan_timeout_s
        self._latest: Optional[Sample] = None
        self._seq: int = 0
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._task: Optional[asyncio.Task[None]] = None
        self._stop_evt: Optional[asyncio.Event] = None

    def start(self) -> None:
        if self._loop is not None:
            return
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop_evt = asyncio.Event()
        self._task = self._loop.create_task(self._async_main())

    def pump(self) -> None:
        if self._loop is None or self._loop.is_closed():
            return
        try:
            self._loop.run_until_complete(asyncio.sleep(0))
        except Exception as exc:
            print(f"[ble-imu] asyncio pump failed: {exc!r}")

    def stop(self) -> None:
        if self._loop is None:
            return
        if self._stop_evt is not None:
            self._stop_evt.set()
        if self._task is not None and not self._task.done():
            try:
                self._loop.run_until_complete(
                    asyncio.wait_for(self._task, timeout=2.0)
                )
            except asyncio.TimeoutError:
                self._task.cancel()
                try:
                    self._loop.run_until_complete(self._task)
                except asyncio.CancelledError:
                    pass
        if not self._loop.is_closed():
            self._loop.close()
        self._loop = None
        self._task = None
        self._stop_evt = None

    def latest(self) -> Optional[Sample]:
        return self._latest

    def latest_with_seq(self) -> Tuple[int, Optional[Sample]]:
        return (self._seq, self._latest)

    def _handle_notification(self, _sender, data: bytearray) -> None:
        sample = parse_imu_packet(bytes(data))
        if sample is None:
            return
        self._latest = sample
        self._seq += 1

    def _matches_target(self, _device: object, advertisement_data: object) -> bool:
        """True when an advertisement is from the MovementTwin firmware."""
        local_name = getattr(advertisement_data, "local_name", None)
        if local_name == self._device_name:
            return True
        service_uuids = getattr(advertisement_data, "service_uuids", ()) or ()
        return IMU_SERVICE_UUID.lower() in {u.lower() for u in service_uuids}

    async def _async_main(self) -> None:
        # Import here so a missing bleak install only breaks the BLE path.
        from bleak import BleakClient, BleakScanner

        try:
            print(
                f"[ble-imu] scanning for {self._device_name!r} "
                f"(service {IMU_SERVICE_UUID}) for {self._scan_timeout:.0f} s..."
            )
            # On macOS, BLEDevice.name is often None during scanning; the name
            # from firmware setLocalName() lives in advertisement local_name.
            # find_device_by_filter waits for that field (see bleak docs).
            target = await BleakScanner.find_device_by_filter(
                self._matches_target,
                timeout=self._scan_timeout,
                service_uuids=[IMU_SERVICE_UUID],
            )

            print(
                f"[ble-imu] connecting to {target.address} "
                f"({target.name or self._device_name})..."
            )
            async with BleakClient(target.address) as client:
                await client.start_notify(IMU_CHAR_UUID, self._handle_notification)
                print("[ble-imu] streaming IMU at ~50 Hz.")
                assert self._stop_evt is not None
                while not self._stop_evt.is_set():
                    await asyncio.sleep(0.05)
                await client.stop_notify(IMU_CHAR_UUID)
        except Exception as exc:
            print(f"[ble-imu] BLE task terminated: {exc!r}")
