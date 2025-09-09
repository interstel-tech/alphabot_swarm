import serial
import time
import json
import redis

# Redis setup
# r = redis.Redis(host='localhost', port=6379, db=0)

# Serial setup with timeout
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
print("Connected to " + DWM.name)

# Initialize UWB in lec mode
DWM.write(b"\r\r")
time.sleep(1)
DWM.write(b"lec\r")
time.sleep(1)

# Start reading loop
missed = 0

try:
    while True:
        print("🔄 Waiting for data...")
        try:
            data = DWM.readline().decode("utf-8").strip()
        except UnicodeDecodeError as e:
            print("⚠️ Decode error:", e)
            continue

        if not data:
            print("⚠️ No data received.")
            missed += 1
        elif "POS" in data:
            try:
                parts = data.split(",")
                x = parts[parts.index("POS")+1]
                y = parts[parts.index("POS")+2]
                pos = {"x": x, "y": y}
                pos_json = json.dumps(pos)
                print("✅", pos_json)
                # r.set("pos", pos_json)
                missed = 0
            except Exception as e:
                print("⚠️ Parse error:", e)
                missed += 1
        else:
            print("⚠️ Non-position data:", data)
            missed += 1

        # Auto-recover if UWB goes silent
        if missed > 5:
            print("🔁 Re-sending lec command to recover...")
            DWM.write(b"\r\r")
            time.sleep(1)
            DWM.write(b"lec\r")
            time.sleep(1)
            missed = 0

except KeyboardInterrupt:
    print("🛑 Stopping...")
    DWM.write(b"\r")
    DWM.close()
