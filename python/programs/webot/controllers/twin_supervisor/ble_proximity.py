"""Thread-safe proximity source abstraction for the Webots Supervisor.

The Webots controller loop is synchronous (`supervisor.step(timestep)`), but
`bleak` is built on `asyncio`. To keep the controller loop responsive, this
module runs the BLE scan + notification handler in a daemon background thread
and exposes the most recent APDS9960 reading via `latest()`.

UUIDs and the device name mirror the firmware in
`firmware/CollisionDetection/src/main.cpp` and the existing host script in
`python/programs/collisionDetector/collisionDetector.py`.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Optional

DEVICE_NAME = "PhysicalTwin"
PROXIMITY_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
INSTRUCTION_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

class ProximitySource:
    """Interface implemented by `BleProximitySource` and `MockProximitySource`."""

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def latest(self) -> Optional[int]:
        """Most recent APDS9960 reading in [0, 255], or None if not ready."""
        raise NotImplementedError


class MockProximitySource(ProximitySource):
    """Deterministic source for offline development.

    Returns a cosine sweep across [0, 255] so the obstacle visibly slides in and
    out without needing the physical Nano powered on.
    """

    def __init__(self, period_s: float = 8.0) -> None:
        self._period = period_s
        self._t0 = time.time()

    def start(self) -> None:
        self._t0 = time.time()

    def stop(self) -> None:
        pass

    def latest(self) -> Optional[int]:
        phase = (time.time() - self._t0) / self._period * 2.0 * math.pi
        return int(round((math.cos(phase) + 1.0) / 2.0 * 255.0))


class BleProximitySource(ProximitySource):
    """Connects to the Nano 33 BLE PhysicalTwin in a background thread."""

    def __init__(
        self,
        device_name: str = DEVICE_NAME,
        scan_timeout_s: float = 5.0,
    ) -> None:
        self._client = None
        self._device_name = device_name
        self._scan_timeout = scan_timeout_s
        self._latest: Optional[int] = None
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="ble-proximity"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def latest(self) -> Optional[int]:
        with self._lock:
            return self._latest

    def _handle_notification(self, _sender, data: bytearray) -> None:
        if not data:
            return
        if int(data[0]) < 100:
            print("Too close! Requesting Nano to flash red light...")
            asyncio.create_task(
                self._client.write_gatt_char(INSTRUCTION_UUID, "flash".encode(), response=True)
            )
        with self._lock:
            self._latest = int(data[0])

    def _run(self) -> None:
        try:
            asyncio.run(self._async_main())
        except Exception as exc:  # controller must keep running on BLE errors
            print(f"[ble] thread terminated: {exc!r}")

    async def _async_main(self) -> None:
        # Import inside the thread so a missing bleak install only breaks the
        # BLE path, not `--mock` runs.
        from bleak import BleakClient, BleakScanner

        print(f"[ble] scanning for {self._device_name!r}...")
        devices = await BleakScanner.discover(timeout=self._scan_timeout)
        target = next((d for d in devices if d.name == self._device_name), None)
        if target is None:
            print(f"[ble] {self._device_name!r} not found; staying idle.")
            return

        print(f"[ble] connecting to {target.address}...")
        async with BleakClient(target.address) as client:
            self._client = client
            await client.start_notify(PROXIMITY_UUID, self._handle_notification)
            print("[ble] streaming proximity. Ctrl+C in Webots to stop.")
            while not self._stop_evt.is_set():
                await asyncio.sleep(0.1)
            await client.stop_notify(PROXIMITY_UUID)
