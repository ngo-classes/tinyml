import asyncio
from xmlrpc import client
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "PhysicalTwin"

proximity_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"  # Nano -> computer
instruction_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"  # computer -> Nano

# Global reference to client so the notification handler can talk back
ble_client = None

def handle_notification(sender, data):
    global ble_client
    if not ble_client:
        return

    # APDS9960 outputs a single unsigned byte (0-255)
    proximity_value = data[0]
    #print(f"Nano Proximity: {proximity_value}")

    # If an object gets too close (< 100), issue an instruction packet
    if proximity_value < 100:
        print("Too close! Requesting Nano to flash red light...")
        asyncio.create_task(
            ble_client.write_gatt_char(instruction_UUID, "flash".encode(), response=True)
        )

async def main():
    global ble_client
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = None
    for d in devices:
        print(d.name, d.address)
        if d.name == DEVICE_NAME:
            target = d
            break

    if target is None:
        print("Could not find our physical twin.")
        return

    print(f"Connecting to {target.name}...")
    async with BleakClient(target.address) as client:
        ble_client = client
        print("Connected.")

        await client.start_notify(proximity_UUID, handle_notification)
        print("System active. Monitoring proximity stream. Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopping digital twin link.")