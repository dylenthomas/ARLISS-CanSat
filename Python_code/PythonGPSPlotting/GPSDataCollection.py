# Need to see how noisy the gps positional data is like velocity, position, heading, etc.
# This tool will collect data from a serial port where the Arduino is sending GPS data in a format,
#   then pasting it to a csv file

# Can use this website to see gps data on a map https://plotgps.com/

import serial
import pandas as pd
import numpy as np
import datetime
import time

def convertGPSSerialData(raw_data, is_header=False):
    raw_data = raw_data.decode("utf-8")
    raw_data = raw_data.replace('\n', '')
    print(raw_data)
    raw_data = raw_data.split(",")

    processedData = []
    for data in raw_data:
        if data != '' and not is_header:
            processedData.append(float(data))
        elif is_header:
            processedData.append(data)
        else:
            return None

    if not is_header:
        processedData = np.array(processedData)
        return np.expand_dims(processedData, 0)
    else:
        return processedData

if __name__ == "__main__":
    arduinoSerial = serial.Serial('COM8', baudrate=9600, bytesize=8, timeout=5)
    print("Connected to serial port")

    try:
        print("\tStarting data collection...")
        print("\tPress Ctrl-C to stop collecting data and save to a csv file.")
        time.sleep(1) # let user read the message

        print("\tWaiting for header...")
        csvHeader = arduinoSerial.read_until()
        csvProcessedHeader = convertGPSSerialData(csvHeader, True)

        if csvProcessedHeader is not None and "time" in csvProcessedHeader:
            allGPSData = np.empty((1, len(csvProcessedHeader)))
        else:
            raise ValueError

        while True:
            gpsData = arduinoSerial.read_until()
            gpsProcessedData = convertGPSSerialData(gpsData)
            if gpsProcessedData is not None:
                allGPSData = np.concatenate((allGPSData, gpsProcessedData))
    except KeyboardInterrupt:
        print("Stopping data collection,")
        print("Saving data...")
        current_time = datetime.datetime.now()
        file_name = f"out/GPS_Data_{current_time.month}_{current_time.day}_{current_time.hour}_{current_time.minute}_{current_time.second}.csv"

        gpsDataFrame = pd.DataFrame(allGPSData, columns=csvProcessedHeader)
        gpsDataFrame.to_csv(file_name)
    except ValueError:
        print("There was an issue reading the csv header,")
        print("Restart the arduino by pressing the reset button and run this script again.")