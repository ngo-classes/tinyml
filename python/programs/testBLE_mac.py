"""List BLE devices visible to this Mac (diagnostic for MovementTwin / Webots).

Run from the same Python env as Webots (e.g. conda activate tinyml):

    python python/programs/testBLE_mac.py

Optional: pass a scan duration in seconds (default 12):

    python python/programs/testBLE_mac.py 15
"""

from __future__ import annotations

import asyncio
import sys
from typing import Optional

from bleak import BleakScanner

# Match firmware/MovementDetection and movement_supervisor/ble_imu.py
MOVEMENT_TWIN_NAME = "MovementTwin"
MOVEMENT_SERVICE_UUID = "19B10010-E8F2-537E-4F6C-D104768A1214"


def _fmt_uuids(uuids: list[str]) -> str:
    if not uuids:
        return "(none)"
    return ", ".join(uuids)


async def scan_and_list(timeout_s: float) -> None:
    print(f"Scanning for {timeout_s:.0f} s (return_adv=True)...\n")

    discovered = await BleakScanner.discover(
        timeout=timeout_s,
        return_adv=True,
    )

    rows: list[tuple[str, str, Optional[str], str, str]] = []
    movement_hits: list[str] = []

    for address, (device, adv) in sorted(
        discovered.items(),
        key=lambda item: (item[1][1].local_name or item[1][0].name or "").lower(),
    ):
        os_name = device.name
        local_name = adv.local_name
        services = _fmt_uuids(list(adv.service_uuids))
        rssi = adv.rssi if adv.rssi is not None else "?"
        label = local_name or os_name or "(no name)"
        rows.append((label, address, local_name, os_name or "", services, str(rssi)))

        name_match = local_name == MOVEMENT_TWIN_NAME or os_name == MOVEMENT_TWIN_NAME
        svc_match = MOVEMENT_SERVICE_UUID.lower() in {
            u.lower() for u in adv.service_uuids
        }
        if name_match or svc_match:
            movement_hits.append(address)

    print(f"Found {len(rows)} device(s):\n")
    print(
        f"{'#':>3}  {'label':<28}  {'address':<38}  {'rssi':>4}  "
        f"{'local_name':<16}  {'os_name':<16}  service_uuids"
    )
    print("-" * 120)

    for i, (label, address, local_name, os_name, services, rssi) in enumerate(
        rows, start=1
    ):
        mark = " *" if address in movement_hits else ""
        print(
            f"{i:3}{mark}  {label:<28}  {address:<38}  {rssi:>4}  "
            f"{(local_name or '-'):<16}  {(os_name or '-'):<16}  {services}"
        )

    print()
    if movement_hits:
        print(
            f"MovementTwin candidate(s) ({len(movement_hits)}): "
            + ", ".join(movement_hits)
        )
    else:
        print(f"No device advertised {MOVEMENT_TWIN_NAME!r} or {MOVEMENT_SERVICE_UUID}.")

    print("\n--- find_device_by_name (same filter bleak recommends) ---")
    by_name = await BleakScanner.find_device_by_name(
        MOVEMENT_TWIN_NAME,
        timeout=timeout_s,
        service_uuids=[MOVEMENT_SERVICE_UUID],
    )
    print(f"find_device_by_name: {by_name}")

    print("\n--- find_device_by_filter (name or IMU service) ---")
    by_filter = await BleakScanner.find_device_by_filter(
        lambda _d, ad: (
            ad.local_name == MOVEMENT_TWIN_NAME
            or MOVEMENT_SERVICE_UUID.lower()
            in {u.lower() for u in ad.service_uuids}
        ),
        timeout=timeout_s,
        service_uuids=[MOVEMENT_SERVICE_UUID],
    )
    print(f"find_device_by_filter: {by_filter}")

    if not movement_hits and by_name is None and by_filter is None:
        print(
            "\nIf the Nano Serial says 'BLE advertising as MovementTwin' but nothing "
            "appears here:\n"
            "  • Power-cycle the board; close other BLE apps (Webots, collisionDetector).\n"
            "  • Only one central can be connected — connected boards stop advertising.\n"
            "  • System Settings → Privacy → Bluetooth: allow Python (this interpreter).\n"
            "  • Keep the board within ~1 m; USB Serial does not prove the radio is heard."
        )


def main() -> None:
    timeout_s = 12.0
    if len(sys.argv) > 1:
        timeout_s = float(sys.argv[1])
    asyncio.run(scan_and_list(timeout_s))


if __name__ == "__main__":
    main()
