import socket 
import RPi.GPIO as GPIO 
import time 
import math 
import sys
import os

import serial
import time
import json
import redis


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from AlphaBot2 import AlphaBot2

# Initialize AlphaBot2
Ab = AlphaBot2()
Ab.setPWMA(20)  # Set motor speed to 20%
Ab.setPWMB(20)  # Set motor speed to 20%

# UDP Client Configuration
CLIENT_IP = "192.168.33.222"
SERVER_IP = "192.168.33.249"
PORT = 12000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind((CLIENT_IP, PORT))  # Bind to receive messages

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

print("Waiting for movement commands...")

x_position = 0
y_position = 0

# Function to send periodic position updates
def send_position_update():
    global x_position, y_position
    position_message = f"{x_position},{y_position}"
    client_socket.sendto(position_message.encode(), (SERVER_IP, PORT))

data, addr = client_socket.recvfrom(1024)
y, x = map(int, data.decode().split(","))  # Swapped x and y
print(f"Received: x={x} cm, y={y} cm")

angle = math.degrees(math.atan2(x,y))
print(f"Turning to {angle:.2f} degrees")

if angle > 0:
    Ab.right()
else:
    Ab.left()

time.sleep(abs(angle)/360)
Ab.stop()
time.sleep(1)

try:
    while x_position != x or y_position != y:
        Ab.forward()
        data = DWM.readline()
        data = data.decode("utf-8").strip()
        if data:
            if "DIST" in data and "AN0" in data and "AN1" in data and "AN2" in data:
                data = data.replace("\r\n", "").split(",")
            
            if "DIST" in data:
                next_value = data[data.index("DIST") + 1]
                if next_value.isdigit():
                    anchor_Number = int(next_value)
                    for i in range(anchor_Number):
                        pos_AN = {
                            "id": data[data.index("AN"+str(i))+1],
                            "x": data[data.index("AN"+str(i))+2],
                            "y": data[data.index("AN"+str(i))+3],
                            "dist": data[data.index("AN"+str(i))+5]
                        }
                        pos_AN = json.dumps(pos_AN)
                        r.set('AN'+str(i), pos_AN)
                else:
                    print(f"Unexpected value after 'DIST': {next_value}")

            if "POS" in data:
                pos = {
                    "x": data[data.index("POS") + 1],
                    "y": data[data.index("POS") + 2]
                }
                pos = json.dumps(pos)
                print(pos)
                time.sleep(0.5)
                r.set("pos", pos)
    Ab.stop()
    print("Movement complete")

except KeyboardInterrupt:
    print("Client shutting down.")
    DWM.write("\r".encode())
    DWM.close()
finally:
    client_socket.close()
    GPIO.cleanup()
    print("GPIO cleanup completed.")
