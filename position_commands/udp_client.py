import socket 
import RPi.GPIO as GPIO 
import time 
import math 
import sys
import os

import serial
import json
import redis


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from AlphaBot2 import AlphaBot2

# Initialize AlphaBot2
Ab = AlphaBot2()
Ab.setPWMA(20)  # Set motor speed
Ab.setPWMB(20)  # Set motor speed

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

# Receive target position from UDP command BEFORE getting initial position
data, addr = client_socket.recvfrom(1024)
y_target, x_target = map(float, data.decode().split(","))  # Swapped x and y
print(f"Received Target: x={x_target} m, y={y_target} m")

# Retrieve initial position from UWB **after receiving the command**
x_position, y_position = None, None
while True:
    data = DWM.readline().decode("utf-8").strip()
    if "POS" in data:
        parsed_data = data.replace("\r\n", "").split(",")
        try:
            x_position = float(parsed_data[parsed_data.index("POS") + 1])
            y_position = float(parsed_data[parsed_data.index("POS") + 2])
            print(f"Initial Position: x={x_position}, y={y_position}")
            break  # Exit loop once we get the initial position
        except (ValueError, IndexError):
            print("Error parsing initial position, retrying...")

# Calculate angle to turn based on initial position
angle = math.degrees(math.atan2(y_target - y_position, x_target - x_position))
print(f"Turning to {angle:.2f} degrees")

if angle > 0:
    Ab.right()
else:
    Ab.left()

time.sleep(abs(angle) / 360)  # Turning delay based on angle
Ab.stop()

try:
    while abs(x_position - x_target) > 0.1 or abs(y_position - y_target) > 0.1:
        Ab.forward()
        
        # Update position from UWB
        data = DWM.readline().decode("utf-8").strip()
        if "POS" in data:
            parsed_data = data.replace("\r\n", "").split(",")
            try:
                x_position = float(parsed_data[parsed_data.index("POS") + 1])
                y_position = float(parsed_data[parsed_data.index("POS") + 2])
                print(f"Updated Position: x={x_position}, y={y_position}")
                time.sleep(0.5)
            except (ValueError, IndexError):
                print("Error parsing position update")

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
