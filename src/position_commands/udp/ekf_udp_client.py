import socket
import RPi.GPIO as GPIO
import time
import math
import sys
import os
import serial
import redis
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from AlphaBot2 import AlphaBot2

# --- EKF Helpers ---
def state_transition(x, dt):
    x_pos, y_pos, v_forward, yaw = x
    x_pos += v_forward * math.cos(yaw) * dt
    y_pos += v_forward * math.sin(yaw) * dt
    return np.array([x_pos, y_pos, v_forward, yaw])

def jacobian_F(x, dt):
    _, _, v_forward, yaw = x
    return np.array([
        [1, 0, math.cos(yaw)*dt, -v_forward * math.sin(yaw) * dt],
        [0, 1, math.sin(yaw)*dt,  v_forward * math.cos(yaw) * dt],
        [0, 0, 1,                0],
        [0, 0, 0,                1]
    ])

def measurement_function(x): return x[:2]
def jacobian_H(x): return np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

# --- Setup ---
Ab = AlphaBot2()
Ab.setPWMA(20)
Ab.setPWMB(20)

CLIENT_IP = "192.168.33.228"
SERVER_IP = "192.168.33.249"
PORT = 12000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind((CLIENT_IP, PORT))

r = redis.Redis(host='localhost', port=6379, db=0)
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " + DWM.name)
DWM.write(b"\r\r")
time.sleep(1)
DWM.write(b"lec\r")
time.sleep(1)

# --- EKF Setup ---
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
ekf.x = np.array([0.0, 0.0, 0.0, 0.0])
ekf.P *= 5
ekf.R *= 0.1
ekf.Q *= 0.01
yaw = 0.0

# --- Get Target and Initial Position ---
print("Waiting for movement commands...")
data, addr = client_socket.recvfrom(1024)
y_target, x_target = map(float, data.decode().split(","))
print(f"Target received: x={x_target}, y={y_target}")

x_position, y_position = None, None
while True:
    line = DWM.readline().decode("utf-8").strip()
    if "POS" in line:
        try:
            parts = line.split(",")
            x_position = float(parts[parts.index("POS")+1])
            y_position = float(parts[parts.index("POS")+2])
            ekf.x[0], ekf.x[1] = x_position, y_position
            print(f"Initial Position: x={x_position}, y={y_position}")
            break
        except Exception:
            continue

# --- Turn to Face Target ---
angle_to_target = math.degrees(math.atan2(y_target - y_position, x_target - x_position))
print(f"Turning to {angle_to_target:.2f} degrees")

if angle_to_target > 0:
    Ab.right()
else:
    Ab.left()

time.sleep(abs(angle_to_target) / 360)
Ab.stop()

# --- Movement Loop ---
try:
    last_time = time.time()
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Simulated constant forward velocity (e.g., 0.1 m/s)
        ekf.x[2] = 0.1

        # Predict motion
        ekf.x = state_transition(ekf.x, dt)
        ekf.F = jacobian_F(ekf.x, dt)
        ekf.predict()

        # UWB Update
        line = DWM.readline().decode("utf-8").strip()
        if "POS" in line:
            try:
                parts = line.split(",")
                x = float(parts[parts.index("POS")+1])
                y = float(parts[parts.index("POS")+2])
                ekf.update(np.array([x, y]), jacobian_H, measurement_function)
            except Exception:
                continue

        # Control
        x_pos, y_pos = ekf.x[0], ekf.x[1]
        print(f"EKF Position: x={x_pos:.2f}, y={y_pos:.2f}, θ={math.degrees(ekf.x[3]):.2f}°")

        # Stop if close to target
        if abs(x_pos - x_target) < 0.1 and abs(y_pos - y_target) < 0.1:
            Ab.stop()
            print("Target reached.")
            break
        else:
            Ab.forward()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    Ab.stop()
    DWM.write(b"\r")
    DWM.close()
    client_socket.close()
    GPIO.cleanup()
    print("Shutdown complete.")
