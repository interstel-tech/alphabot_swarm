import time
import math
import serial
import redis
import hid
import numpy as np
import RPi.GPIO as GPIO
import sys
import os
from ahrs.filters import Madgwick

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from AlphaBot2 import AlphaBot2

# --- CP2112 HID Driver ---
class HIDDriver:
    def __init__(self, *, serial=None):
        self.h = hid.device()
        self.h.open(0x10C4, 0xEA90, serial)
        self.h.send_feature_report([0x03, 0xFF, 0x00, 0x00, 0x00])
        self.h.send_feature_report([0x04, 0x00, 0xFF])
        self.h.send_feature_report([0x04, 0xFF, 0xFF])
        self.h.send_feature_report([0x02, 0x83, 0xFF, 0xFF, 0x01])
        self.h.send_feature_report([0x06, 0x00, 0x01, 0x86, 0xA0, 0x02,
                                    0x00, 0x00, 0xFF, 0x00, 0xFF, 0x01,
                                    0x00, 0x0F])

    def read_byte_data(self, address, register):
        self.h.write([0x11, address << 1, 0x00, 0x01, 0x01, register])
        for _ in range(10):
            self.h.write([0x15, 0x01])
            resp = self.h.read(7)
            if resp and (resp[0] == 0x16) and (resp[2] == 5):
                self.h.write([0x12, 0x00, 0x01])
                resp = self.h.read(4)
                return resp[3]
        return None

    def write_byte_data(self, address, register, value):
        self.h.write([0x14, address << 1, 0x02, register, value])
        time.sleep(0.01)

# --- IMU Helpers ---
def read_word(driver, addr, high_reg, low_reg):
    high = driver.read_byte_data(addr, high_reg)
    low = driver.read_byte_data(addr, low_reg)
    if high is None or low is None:
        return None
    raw = (high << 8) | low
    return raw - 65536 if raw & 0x8000 else raw

def read_accel(driver, addr):
    return {
        "x": read_word(driver, addr, 0x31, 0x32),
        "y": read_word(driver, addr, 0x2F, 0x30),
        "z": read_word(driver, addr, 0x2D, 0x2E)
    }

def read_gyro(driver, addr):
    return {
        "x": read_word(driver, addr, 0x37, 0x38),
        "y": read_word(driver, addr, 0x35, 0x36),
        "z": read_word(driver, addr, 0x33, 0x34)
    }

def convert_units(accel_raw, gyro_raw):
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    GRAVITY = 9.80665
    ax = (accel_raw["x"] / ACCEL_SCALE) * GRAVITY
    ay = (accel_raw["y"] / ACCEL_SCALE) * GRAVITY
    az = (accel_raw["z"] / ACCEL_SCALE) * GRAVITY
    gx = math.radians(gyro_raw["x"] / GYRO_SCALE)
    gy = math.radians(gyro_raw["y"] / GYRO_SCALE)
    gz = math.radians(gyro_raw["z"] / GYRO_SCALE) + 0.019
    return np.array([ax, ay, az]), np.array([gx, gy, gz])

# --- Madgwick Yaw ---
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])

def quaternion_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

# --- Setup ---
Ab = AlphaBot2()
r = redis.Redis(host='localhost', port=6379, db=0)
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
print("Connected to " + DWM.name)
DWM.write(b"\r\r")
time.sleep(1)
DWM.write(b"lec\r")
time.sleep(1)

d = HIDDriver()
imu_addr = 0x69
d.write_byte_data(imu_addr, 0x06, 0x01)
d.write_byte_data(imu_addr, 0x3F, 0x00)

# --- Target Input ---
x_target = float(input("Enter target x (m): "))
y_target = float(input("Enter target y (m): "))

print("Waiting for first UWB reading...")
while True:
    data = DWM.readline().decode("utf-8").strip()
    if "POS" in data:
        parts = data.split(",")
        try:
            x_pos = float(parts[parts.index("POS") + 1])
            y_pos = float(parts[parts.index("POS") + 2])
            break
        except:
            continue

# --- Initial Rotation ---
print("Rotating to target...")
Ab.setPWMA(12)
Ab.setPWMB(12)
target_angle = math.atan2(y_target - y_pos, x_target - x_pos)
yaw = 0.0
last_time = time.time()
if target_angle > 0:
    Ab.left()
else:
    Ab.right()
initial_sign = math.copysign(1, yaw - target_angle)

while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    accel_raw = read_accel(d, imu_addr)
    gyro_raw = read_gyro(d, imu_addr)
    if None in accel_raw.values() or None in gyro_raw.values():
        continue

    acc, gyr = convert_units(accel_raw, gyro_raw)
    q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
    if q is not None:
        yaw = quaternion_to_yaw(q)
    yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7
    angle_error = math.degrees(yaw - target_angle)
    print(f"Yaw Error: {angle_error:.2f}°")
    if abs(angle_error) < 5 or math.copysign(1, angle_error) != initial_sign:
        break
Ab.stop()

# --- Move to Target ---
print("Moving to target...")
Ab.setPWMA(20)
Ab.setPWMB(19)

import json

# Start reading loop
try:
    while True:
        print("🔄 Waiting for data...")
        try:
            data = DWM.readline().decode("utf-8").strip()
        except UnicodeDecodeError as e:
            print("⚠️ Decode error:", e)
            continue

        if "POS" in data:
            try:
                parts = data.split(",")
                x_pos = float(parts[parts.index("POS")+1])
                y_pos = float(parts[parts.index("POS")+2])
                pos = {"x": x_pos, "y": y_pos}
                pos_json = json.dumps(pos)
                print("✅", pos_json)
                r.set("pos", pos_json)
            except Exception as e:
                print("⚠️ Parse error:", e)
            Ab.forward()
        if abs(x_pos - x_target) < 0.1 and abs(y_pos - y_target) < 0.1:
            Ab.stop()
            break

    # --- Final Rotation to 0° ---
    print("Rotating back to 0°...")
    Ab.setPWMA(12)
    Ab.setPWMB(12)
    initial_yaw_sign = math.copysign(1, yaw)
    if yaw > 0:
        Ab.right()
    else:
        Ab.left()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now
        accel_raw = read_accel(d, imu_addr)
        gyro_raw = read_gyro(d, imu_addr)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
        if q is not None:
            yaw = quaternion_to_yaw(q)
        yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7
        print(math.degrees(yaw))
        if abs(math.degrees(yaw)) < 5 or math.copysign(1, yaw) != initial_yaw_sign:
            break
    Ab.stop()

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    DWM.write(b"\r")
    DWM.close()
    GPIO.cleanup()
