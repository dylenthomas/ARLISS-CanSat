from bleak.exc import BleakError
from bleak import BleakClient, BleakScanner
from datetime import datetime

import asyncio
import sys
import time
import csv

uuids = ["19b10000-e8f2-537e-4f6c-d104768a1214", 
         "19b10001-e8f2-537e-4f6c-d104768a1214",
         "19b10002-e8f2-537e-4f6c-d104768a1214"]

BLEname = "Nano33BLE"

csv_filename = f"data/CanSat_currentdraw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_filename, mode='w', newline='')

scanner = BleakScanner(service_uuids=uuids)

printdelay = 1

notif_q = asyncio.Queue()

current_collection_command = """
To start current data collection send:
    x.x_y.y

Where-
    x.x is a float between -1 and 1 specifying duty cycle and direction of the first motor
    y.y is a float between -1 and 1 specifying duty cycle and direction of the second motor

Send "Start data collection" to confirm motor speeds and start transmitting data
"""

async def async_input(prompt):
    return await asyncio.get_event_loop().run_in_executor(None, input, prompt)

async def handleNotification(sender, data):
    await notif_q.put(data.decode("utf-8").strip())

async def main():
    print(f"Looking for {BLEname}...")
    device = None

    # first try to find the arduino by its name
    device = await scanner.find_device_by_name(BLEname)
    if device is not None:
        print(f"\tFound {BLEname} as: {device}")

    else:
        # if the arduino wasn't found by manually searching for the name, try scanning one more time then searching
        print(f"Could not find {BLEname} by name,")
        print("Manually scanning for devices...")

        devices = await scanner.discover()
        for device in devices:
            if BLEname in str(device.name):
                print(f"Found {BLEname} as: {device}")
                device = device

    if device is None:
        print(f"Could not connect to {BLEname}")
        return

    async with BleakClient(address_or_ble_device=device) as client:
            print("="*30)
            for service in client.services:
                print(f"\t\tService: {service.uuid}")
                for char in service.characteristics:
                    print(f"\t\t  Characteristic: {char.uuid}, props: {char.properties}")
            print("="*30)

            print(f"\tConnected to {device.address}")
            await client.start_notify(uuids[1], handleNotification)

            while True:
                notification = await notif_q.get()
                print(notification)
                if "<CONNECTED>" in notification:
                    break

            while True:
                notification = await notif_q.get()
                print(notification)
                if "<READY>" in notification:
                    break

            print(current_collection_command)

            while True:
                cmd = await async_input("Enter command: ")
                cmd += '\0'

                print(f"\tSending Command to {BLEname}")
                for c in cmd:
                    c = c.encode('utf-8')
                    await client.write_gatt_char(uuids[2], c, response=True)

                if cmd[:-1] == "Start data collection":
                    notification = await notif_q.get()
                    await asyncio.sleep(2)
                    print(notification)
                    break
                
            
            csv_writer = None 
            headers = ["time", "current"]
            
            while True: 
                notification = await notif_q.get()
            
                try:
                    values = notification.split(',')
                    print(values)

                    if csv_writer is None:
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow(headers)

                    csv_writer.writerow(values)

                except Exception as e:
                    print(f"[ERROR] Failed to write row: {e}")
                
if __name__ == "__main__":
    import os
    os.system('cls' if os.name == 'nt' else 'clear')

    print("=" * 30)
    print("   Bluetooth Device Info")
    print("=" * 30)
    print(f"Name : {BLEname}")
    print(f"UUID(s) : {uuids}")
    print("=" * 30)
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt: 
        csv_file.close()
        print(f"[INFO] CSV file saved: {csv_filename}")
        print("Stopping...")



