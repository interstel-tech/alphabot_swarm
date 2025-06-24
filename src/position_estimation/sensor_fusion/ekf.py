import time
import math
import hid
import numpy as np
import imufusion
from networktables import NetworkTables
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))
from utils.dwm import dwm1001
from utils.kalman import Fusion, UWB

### === IMU Code (Embedded with Calibration) === ###

class HIDDriver:
    def __init__(self, *, serial=None, led=True):
        h = self.h = hid.device()
        self.h.open(0x10C4, 0xEA90, serial)
        print("Manufacturer:", h.get_manufacturer_string())
        print("Product:", h.get_product_string())
        print("Serial No:", h.get_serial_number_string())

        print('Blink LED and set GPIO')
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
    print("🔧 Initializing ICM-20948...")
    driver.write_byte_data(addr, 0x06, 0x01)
    time.sleep(0.01)
    driver.write_byte_data(addr, 0x3F, 0x00)
    time.sleep(0.01)
    print("✅ Initialization complete.")

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
    gz = math.radians(gyro_raw["gyro_z"] / GYRO_SCALE) + 0.019
    return np.array([gx, gy, gz])

### === Main Execution === ###

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

print("🔌 Connecting to DWM1001...")
dwm = dwm1001("/dev/ttyACM0")

print("🔌 Connecting to IMU...")
driver = HIDDriver()
imu_address = 0x69
initialize_icm20948(driver, imu_address)

print("⌛ Initializing AHRS (IMU orientation)...")
ahrs = imufusion.Ahrs()
prev_time = time.monotonic_ns()
while ahrs.flags.initialising:
    gyro_raw = read_gyro(driver, imu_address)
    if None in gyro_raw.values():
        continue
    gyr = convert_gyro(gyro_raw)
    dt = (time.monotonic_ns() - prev_time) / 1e9
    ahrs.update_no_magnetometer(gyr, np.array([0.0, 0.0, 9.8]), dt)
    prev_time = time.monotonic_ns()
print("✅ AHRS Ready.")

print("📰 Waiting for initial UWB position...")
init_pos = None
while init_pos is None:
    try:
        init_pos = dwm.position()
        print(f"Initial UWB: x={init_pos.px}, y={init_pos.py}")
    except:
        print("Retrying UWB position...")
        time.sleep(0.5)

plain_ekf = UWB(init_pos.px, init_pos.py, init_pos.pz)
fusion_ekf = Fusion(init_pos.px, init_pos.py, init_pos.pz)

prev_imu = time.monotonic()
prev_dwm = time.monotonic()
anchors = []

print("🚀 Starting loop (gyro + UWB fusion)...")
try:
    while True:
        now = time.monotonic()

        if now - prev_dwm >= 0.1:
            try:
                anchors = dwm.anchors()
            except Exception as e:
                print(f"❌ Failed to get anchors: {e}")
            prev_dwm = now

        if now - prev_imu >= 0.005:
            gyro_raw = read_gyro(driver, imu_address)
            if None in gyro_raw.values():
                print("⚠️ Gyro read failed. Skipping.")
                continue

            gyr = convert_gyro(gyro_raw)
            dt = now - prev_imu
            ahrs.update_no_magnetometer(gyr, np.array([0.0, 0.0, 9.8]), dt)

            yaw = ahrs.quaternion.to_euler()[2]
            print(f"Yaw: {math.degrees(yaw):.2f}°")

            plain_ekf.dwm_update(anchors)
            fusion_ekf.dwm_update(anchors, 0, 0, 0)  # No accel input

            prev_imu = now

            print(f"UWB: ({plain_ekf.get_x()[0,0]:.2f}, {plain_ekf.get_x()[1,0]:.2f})")
            print(f"Fusion: ({fusion_ekf.get_x()[0,0]:.2f}, {fusion_ekf.get_x()[1,0]:.2f})")
            print("-" * 50)

            nt.putNumber('dwm_x', round(plain_ekf.get_x()[0, 0], 2))
            nt.putNumber('dwm_y', round(plain_ekf.get_x()[1, 0], 2))
            nt.putNumber('fuse_x', round(fusion_ekf.get_x()[0, 0], 2))
            nt.putNumber('fuse_y', round(fusion_ekf.get_x()[1, 0], 2))

except KeyboardInterrupt:
    print("🛑 Shutting down...")
    dwm.close()
    driver.h.close()
