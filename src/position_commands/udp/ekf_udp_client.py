import time
import math
import serial
import redis
import hid
import numpy as np
import RPi.GPIO as GPIO
import sys
import os
import json
from ahrs.filters import Madgwick
import socket

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
from AlphaBot2 import AlphaBot2
from utils.kalman import Fusion
import imufusion

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

def quaternion_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

def calibrate_imu(driver, imu_address):
    print("⌛ Performing AHRS-based IMU calibration...")
    ahrs = imufusion.Ahrs()
    prev_time = time.monotonic_ns()
    offset = 350000
    grav_base = np.array([[0], [0], [1]])

    while ahrs.flags.initialising:
        accel_raw = read_accel(driver, imu_address)
        gyro_raw = read_gyro(driver, imu_address)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        dt = (time.monotonic_ns() - prev_time + offset) / 1e9
        ahrs.update_no_magnetometer(gyr, acc, dt)
        prev_time = time.monotonic_ns()

    print("✅ AHRS initialized. Calibrated.")
    return ahrs

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

ahrs = calibrate_imu(d, imu_addr)

# Initialize Fusion EKF
from utils.kalman import Fusion
print("Waiting for first UWB position for EKF...")
while True:
    line = DWM.readline().decode("utf-8").strip()
    if "POS" in line:
        parts = line.split(",")
        try:
            x0 = float(parts[parts.index("POS") + 1])
            y0 = float(parts[parts.index("POS") + 2])
            break
        except:
            continue
fusion_ekf = Fusion(x0, y0, 0.0)

print("System initialized.")

try:
    while True:
        # UDP setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("192.168.33.228", 12000))

        print("📡 Waiting for target...")
        data, _ = sock.recvfrom(1024)
        y_target, x_target = map(float, data.decode().split(","))
        print(f"🎯 Target: x={x_target}, y={y_target}")

        x_list, y_list = [], []
        while len(x_list) < 20:
            line = DWM.readline().decode("utf-8").strip()
            if "POS" in line:
                try:
                    parts = line.split(",")
                    x = float(parts[parts.index("POS") + 1])
                    y = float(parts[parts.index("POS") + 2])
                    x_list.append(x)
                    y_list.append(y)
                except:
                    continue

        x_pos = sum(x_list) / len(x_list)
        y_pos = sum(y_list) / len(y_list)
        print(f"📍 Starting pos: x={x_pos:.2f}, y={y_pos:.2f}")

        # Rotate to target
        target_angle = math.atan2(y_target - y_pos, x_target - x_pos)
        Ab.left() if target_angle > 0 else Ab.right()

        yaw = 0.0
        last_time = time.time()
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now
            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)
            if None in accel_raw.values() or None in gyro_raw.values():
                continue
            acc, gyr = convert_units(accel_raw, gyro_raw)
            ahrs.update_no_magnetometer(gyr, acc, dt)
            yaw = ahrs.quaternion.to_euler()[2]
            fusion_ekf.dwm_update([], acc[0], acc[1], acc[2])
            angle_error = math.degrees(yaw - target_angle)
            if abs(angle_error) < 5:
                break
        Ab.stop()

        print("Moving to target...")
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)
            if None in accel_raw.values() or None in gyro_raw.values():
                continue
            acc, gyr = convert_units(accel_raw, gyro_raw)
            ahrs.update_no_magnetometer(gyr, acc, dt)
            yaw = ahrs.quaternion.to_euler()[2]

            fusion_ekf.dwm_update([], acc[0], acc[1], acc[2])
            x_pos, y_pos = fusion_ekf.get_x()[0, 0], fusion_ekf.get_x()[1, 0]
            r.set("pos", json.dumps({"x": x_pos, "y": y_pos}))
            print(f"Fusion pos: x={x_pos:.2f}, y={y_pos:.2f}")

            if abs(x_pos - x_target) < 0.05 and abs(y_pos - y_target) < 0.05:
                Ab.stop()
                break

            dynamic_target_angle = math.atan2(y_target - y_pos, x_target - x_pos)
            yaw_error = math.degrees(yaw - dynamic_target_angle)
            if yaw_error > 10:
                Ab.right()
            elif yaw_error < -10:
                Ab.left()
            else:
                Ab.forward()

        Ab.stop()

except KeyboardInterrupt:
    print("Stopped.")
finally:
    DWM.write(b"\r")
    DWM.close()
    GPIO.cleanup()