import serial
import time
import json
import redis

# Initialize Redis
r = redis.Redis(host='localhost', port=6379, db=0)

# Initialize Serial Connection
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " + DWM.name)

# Send Commands to Start Data Stream
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)

try:
    while True:
        # Read and decode data
        data = DWM.readline().decode("utf-8").strip()

        if data and "POS" in data:  # Only process position data
            data = data.replace("\r\n", "").split(",")

            pos = {"x": data[data.index("POS")+1], "y": data[data.index("POS")+2]}
            pos_json = json.dumps(pos)

            print(pos_json)  # Debugging output
            r.set("pos", pos_json)  # Store in Redis

        time.sleep(0.5)  # Update every 0.5 seconds

except KeyboardInterrupt:
    print("Stopping...")
    DWM.write("\r".encode())
    DWM.close()
