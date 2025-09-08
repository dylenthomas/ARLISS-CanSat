import serial
import tkinter as tk
import math

# Open Arduino serial port
ser = serial.Serial('COM3', 19200)

# Store last valid target
last_target_lat = 0.0
last_target_lon = 0.0
bad_count = 0
BAD_LIMIT = 10

def bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360) % 360

def update():
    global last_target_lat, last_target_lon, bad_count
    line = ser.readline().decode(errors="ignore").strip()

    try:
        heading, myLat, myLon, tgtLat, tgtLon = map(float, line.split(","))

        # check for invalid target
        if tgtLat == -1 and tgtLon == -1:
            bad_count += 1
            if bad_count >= BAD_LIMIT:
                last_target_lat = -1
                last_target_lon = -1
        else:
            # good target, reset bad_count
            last_target_lat = tgtLat
            last_target_lon = tgtLon
            bad_count = 0

        # use last valid coordinates if current is -1,-1
        displayLat = last_target_lat
        displayLon = last_target_lon

        # bearing from my position to target
        brng = bearing(myLat, myLon, displayLat, displayLon)
        heading /= 100000.0

        # difference between target bearing and heading
        diff = (brng - heading + 360) % 360

        # draw arrow based on difference
        canvas.delete("arrow")
        canvas.update_idletasks()  # ensure canvas sizes are correct
        cx = canvas.winfo_width() // 2
        cy = canvas.winfo_height() // 2
        x2 = cx + 200 * math.cos(diff)
        y2 = cy + 200 * math.sin(diff)
        canvas.create_line(cx, cy, x2, y2, arrow=tk.LAST, width=5, fill="red", tags="arrow")

        label_bearing.config(text=f"Brng: {brng:.3f}°, Head: {heading:.1f}°, Diff: {diff:.1f}°")
        label_packet.config(text=f"Packet: {line}")

    except:
        label_rssi.config(text=f"RSSI: {line}")

    root.after(200, update)

root = tk.Tk()
root.title("Direction Finder")

canvas = tk.Canvas(root, width=960, height=540, bg="white")
canvas.pack()

label_bearing = tk.Label(root, text="Bearing: --", font=("Arial", 24))
label_bearing.pack()

label_packet = tk.Label(root, text="Packet: --", font=("Arial", 18), fg="blue")
label_packet.pack()

label_rssi = tk.Label(root, text="RSSI: --", font=("Arial", 18), fg="red")
label_rssi.pack()

update()
root.mainloop()
