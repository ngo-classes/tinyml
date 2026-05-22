import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "TinyML-Nano33"

TX_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"  # Nano -> computer
RX_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"  # computer -> Nano

def handle_notification(sender, data):
    print("Nano:", data.decode(errors="ignore"))

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = None
    for d in devices:
        print(d.name, d.address)
        if d.name == DEVICE_NAME:
            target = d
            break

    if target is None:
        print("Could not find TinyML-Nano33.")
        return

    print(f"Connecting to {target.name}...")
    async with BleakClient(target.address) as client:
        print("Connected.")

        await client.start_notify(TX_UUID, handle_notification)

        print("Sending commands. Try: red, green, blue, off, quit")

        while True:
            cmd = input("> ").strip()

            if cmd == "quit":
                break

            await client.write_gatt_char(RX_UUID, cmd.encode())

        await client.stop_notify(TX_UUID)

asyncio.run(main())