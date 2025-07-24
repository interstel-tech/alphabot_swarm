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
import imufusion
from networktables import NetworkTables

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from alphabot.robot import AlphaBot2
from utils.kalman import Fusion, UWB
from utils.dwm import dwm1001

class HIDDriver:
    def __init__(self,*,serial=None, led=True):
        h = self.h = hid.device()
        self.h.open(0x10C4, 0xEA90, serial)
        self.h.send_feature_report([0x03, 0xFF, 0x00, 0x00, 0x00])
        for _ in range(3):
            self.h.send_feature_report([0x04, 0x00, 0xFF])
            time.sleep(0.1)
            self.h.send_feature_report([0x04, 0xFF, 0xFF])
            time.sleep(0.1)
        self.h.send_feature_report([0x02, 0x83, 0xFF, 0xFF, 0x01])
        self.h.send_feature_report([0x06, 0x00, 0x01, 0x86, 0xA0, 0x02, 0x00, 0x00,
                                    0xFF, 0x00, 0xFF, 0x01, 0x00, 0x0F])

    def read_byte_data(self, address, register):
        self.h.write([0x11, address << 1, 0x00, 0x01, 0x01, register])
        for _ in range(10):
            self.h.write([0x15, 0x01])
            response = self.h.read(7)
            if response and response[0] == 0x16 and response[2] == 5:
                self.h.write([0x12, 0x00, 0x01])
                response = self.h.read(4)
                return response[3]
        return None

    def write_byte_data(self, address, register, value):
        self.h.write([0x14, address << 1, 0x02, register, value])
        time.sleep(0.01)

def initialize_icm20948(driver, addr):
    driver.write_byte_data(addr, 0x06, 0x01)
    time.sleep(0.01)
    driver.write_byte_data(addr, 0x3F, 0x00)
    time.sleep(0.01)

def read_accel(driver, imu_address):
    registers = {
        "accel_x": (0x31, 0x32),
        "accel_y": (0x2F, 0x30),
        "accel_z": (0x2D, 0x2E)
    }
    accel_data = {}
    for axis, (high_reg, low_reg) in registers.items():
        high = driver.read_byte_data(imu_address, high_reg)
        low = driver.read_byte_data(imu_address, low_reg)
        if high is None or low is None:
            accel_data[axis] = None
            continue
        raw = (high << 8) | low
        if raw & 0x8000:
            raw = -((65535 - raw) + 1)
        accel_data[axis] = raw
    return accel_data

def read_gyro(driver, imu_address):
    registers = {
        "gyro_x": (0x37, 0x38),
        "gyro_y": (0x35, 0x36),
        "gyro_z": (0x33, 0x34)
    }
    gyro_data = {}
    for axis, (high_reg, low_reg) in registers.items():
        high = driver.read_byte_data(imu_address, high_reg)
        low = driver.read_byte_data(imu_address, low_reg)
        if high is None or low is None:
            gyro_data[axis] = None
            continue
        raw = (high << 8) | low
        if raw & 0x8000:
            raw = -((65535 - raw) + 1)
        gyro_data[axis] = raw
    return gyro_data

def convert_gyro(gyro_raw):
    GYRO_SCALE = 131.0
    gx = math.radians(gyro_raw["gyro_x"] / GYRO_SCALE)
    gy = math.radians(gyro_raw["gyro_y"] / GYRO_SCALE)
    gz = math.radians(gyro_raw["gyro_z"] / GYRO_SCALE)
    return np.array([gx, gy, gz])


def convert_accel(accel_raw):
    ACCEL_SCALE = 16384.0
    ax = accel_raw["accel_x"] / ACCEL_SCALE * 9.8
    ay = accel_raw["accel_y"] / ACCEL_SCALE * 9.8
    az = accel_raw["accel_z"] / ACCEL_SCALE * 9.8
    return np.array([ax, ay, az])

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")
Ab = AlphaBot2()

print("🔌 Connecting to DWM1001...")
dwm = dwm1001("/dev/ttyACM0")

print("🔌 Connecting to IMU...")
driver = HIDDriver()
imu_address = 0x69
initialize_icm20948(driver, imu_address)

yaw = 0.0
print("⏳ Collecting 100 IMU samples for offset calculation...")

gyro_offsets = np.zeros(3)
accel_offsets = np.zeros(3)
samples = 0

while samples < 100:
    gyro_raw = read_gyro(driver, imu_address)
    accel_raw = read_accel(driver, imu_address)
    if gyro_raw is None or accel_raw is None:
        continue

    gyro_offsets += convert_gyro(gyro_raw)
    accel_offsets += convert_accel(accel_raw)
    samples += 1

gyro_offsets /= samples
accel_offsets /= samples
print(f"\n✅ Gyro Offsets (rad/s): {gyro_offsets}")
print(f"✅ Accel Offsets (m/s²): {accel_offsets}")

print("⌛ Initializing AHRS (IMU orientation)...")
ahrs = imufusion.Ahrs()
prev_time = time.monotonic_ns()
while ahrs.flags.initialising:
    gyro_raw = read_gyro(driver, imu_address)
    accel_raw = read_accel(driver, imu_address)
    if gyro_raw is None or accel_raw is None:
        continue
    gyr = convert_gyro(gyro_raw) - gyro_offsets
    acc = convert_accel(accel_raw) - accel_offsets
    dt = (time.monotonic_ns() - prev_time) / 1e9
    ahrs.update_no_magnetometer(gyr, acc, dt)
    prev_time = time.monotonic_ns()
print("✅ AHRS Ready.")

while True:
    user_input = input("Enter target x y (or q to quit): ")
    if user_input.strip().lower() == 'q':
        break

    try:
        x_target, y_target = map(float, user_input.strip().split())
    except:
        print("Invalid input. Enter x y or q.")
        continue

    print("Collecting initial UWB readings for averaging...")
    x_list, y_list = [], []
    while len(x_list) < 20:
        try:
            pos = dwm.position()
            x_list.append(pos.px)
            y_list.append(pos.py)
        except:
            continue

    x_pos = sum(x_list) / len(x_list)
    y_pos = sum(y_list) / len(y_list)

    print(x_pos)
    print(y_pos)
    fusion_ekf = Fusion(x_pos, y_pos, 0.0)
    target_angle = math.atan2(y_target - y_pos, x_target - x_pos)

    print("Rotating to target...")
    Ab.setPWMA(20)
    Ab.setPWMB(20)
    if target_angle > 0:
        Ab.left()
    else:
        Ab.right()

    prev_time = time.monotonic_ns()
    gyro_raw = read_gyro(driver, imu_address)
    gyr = convert_gyro(gyro_raw) - gyro_offsets
    yaw += gyr[2]
    initial_sign = math.copysign(1, yaw - target_angle)
    while True:
        gyro_raw = read_gyro(driver, imu_address)
        accel_raw = read_accel(driver, imu_address)
        if None in gyro_raw.values():
            continue
        gyr = convert_gyro(gyro_raw) - gyro_offsets
        acc = convert_accel(accel_raw) - accel_offsets
        dt = (time.monotonic_ns() - prev_time) / 1e9
        yaw += gyr[2] * dt
        ahrs.update_no_magnetometer(gyr, acc, dt)
        prev_time = time.monotonic_ns()

        angle_error = math.degrees(yaw - target_angle)
        print(f"Yaw Error: {angle_error:.2f}")
        if abs(angle_error) < 5 or math.copysign(1, angle_error) != initial_sign:
            Ab.stop()
            break

    print("Moving to target...")
    while True:
        gyro_raw = read_gyro(driver, imu_address)
        accel_raw = read_accel(driver, imu_address)
        if None in gyro_raw.values() or accel_raw is None:
            continue

        gyr = convert_gyro(gyro_raw) - gyro_offsets
        acc = convert_accel(accel_raw) - accel_offsets

        dt = (time.monotonic_ns() - prev_time) / 1e9
        ahrs.update_no_magnetometer(gyr, acc, dt)
        prev_time = time.monotonic_ns()

        # yaw = ahrs.quaternion.to_euler()[2]
        # Use only forward acceleration (acc_y_body)
        forward_accel = acc[1]  # assuming forward is along body Y

        acc_x_world = forward_accel * math.cos(yaw)
        acc_y_world = forward_accel * math.sin(yaw)

        # Optional: zero out lateral component since we assume no sideways motion
        acc[0] = acc_x_world
        acc[1] = acc_y_world

        yaw += gyr[2] * dt

        anchors = dwm.anchors()
        fusion_ekf.dwm_update(anchors, acc[0], acc[1], 0)
        x_est, y_est = fusion_ekf.get_x()[0, 0], fusion_ekf.get_x()[1, 0]

        print_yaw = math.degrees(yaw)
        print(print_yaw)
        Ab.forward()
        if abs(x_est - x_target) < 0.05 and abs(y_est - y_target) < 0.05:
            Ab.stop()
            break
        
        dynamic_angle = math.atan2(y_target - y_est, x_target - x_est)
        yaw_error = math.degrees(yaw - dynamic_angle)
        print(f"Yaw Drift: {yaw_error:.2f}°, Position: x={x_est:.2f}, y={y_est:.2f}")

        if yaw_error > 10:
            Ab.setPWMA(20)
            Ab.setPWMB(20)
            Ab.right()
        elif yaw_error < -10:
            Ab.setPWMA(20)
            Ab.setPWMB(20)
            Ab.left()
        else:
            Ab.setPWMA(25)
            Ab.setPWMB(25)
            Ab.forward()



print("🛑 Shutting down...")
dwm.close()
driver.h.close()
GPIO.cleanup()
