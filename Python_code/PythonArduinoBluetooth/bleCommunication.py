from bleak.exc import BleakError
from bleak import BleakClient, BleakScanner

import asyncio
import sys
import time

uuids = ["19b10000-e8f2-537e-4f6c-d104768a1214", 
         "19b10001-e8f2-537e-4f6c-d104768a1214",
         "19b10002-e8f2-537e-4f6c-d104768a1214"]

BLEname = "Nano33BLE"

scanner = BleakScanner(service_uuids=uuids)

available_motors = ["1", "2", "A"]
period_range = [2, 1e6]
duty_cycle_range = [0.0, 1.0]
available_directions = ["F", "B"]

printdelay = 1

notif_q = asyncio.Queue()

mtr_cmd_structure = """
To change motor speed settings follow this command format:

    M_F_DC_Dir

Where,
    M - motor number (1, 2, or A for all)
    F - frequency of PWM in Hz
    DC - duty cycle of PWM as a percentage between 0.0 and 1.0
    Dir - direction of the motor with F being forward and B being backward
"""

breakout_commmand = """
To initiaite the breakout sequence for the cage enter:
    breakout_PW_C_T

Where,
    PW - pulse width for motor PWM
    C - the number of 'back-and-forth' cycles to run on the motors
    T - the number of milliseconds to wait before changing motor direction
"""

navigation_command = """
To start autonomous navigatiton send:
    X_Y
    
Where,
    X - desired x coordinates relative to current position in meters
    Y - desired y coordinates relative to current position in meters
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

            #print(mtr_cmd_structure)
            #print(breakout_commmand)
            print(navigation_command)

            while True:
                cmd = await async_input("Enter command: ")
                cmd += '\0'

                print(f"\tSending Command to {BLEname}")
                for c in cmd:
                    c = c.encode('utf-8')
                    await client.write_gatt_char(uuids[2], c, response=True)

                while True:
                    notification = await notif_q.get()
                    print(notification)

                    if "invalid" in notification.lower():
                        break

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
        print()
        print("Stopping...")
