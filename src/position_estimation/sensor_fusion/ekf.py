import time
import math
import hid
import numpy as np
import imufusion
from networktables import NetworkTables
from utils.dwm import dwm1001
from utils.kalman import Fusion, UWB

### === IMU Code (Embedded with Calibration) === ###

class HIDDriver:
    def __init__(self,*,serial=None, led=True):
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
            if (response and response[0] == 0x16 and response[2] == 5):
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

def read_acceleration(driver, imu_address):
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
        low = driver.read_byte_data(imu_address, imu_address)
        if high is None or low is None:
            gyro_data[axis] = None
            continue
        raw = (high << 8) | low
        if raw & 0x8000:
            raw = -((65535 - raw) + 1)
        gyro_data[axis] = raw
    return gyro_data

def convert_units(accel_raw, gyro_raw=None):
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    GRAVITY = 9.80665

    ax = (accel_raw["accel_x"] / ACCEL_SCALE) * GRAVITY
    ay = (accel_raw["accel_y"] / ACCEL_SCALE) * GRAVITY
    az = (accel_raw["accel_z"] / ACCEL_SCALE) * GRAVITY
    acc = np.array([ax, ay, az])

    if gyro_raw is None:
        return acc

    gx = math.radians(gyro_raw["gyro_x"] / GYRO_SCALE)
    gy = math.radians(gyro_raw["gyro_y"] / GYRO_SCALE)
    gz = math.radians(gyro_raw["gyro_z"] / GYRO_SCALE)
    gyr = np.array([gx, gy, gz])

    return acc, gyr

def calibrate_imu(driver, imu_address, sample_rate_hz=200):
    print("⌛ Performing AHRS-based IMU calibration...")
    ahrs = imufusion.Ahrs()
    prev_time = time.monotonic_ns()
    offset = 350000
    grav_base = np.array([[0], [0], [1]])

    while ahrs.flags.initialising:
        accel_raw = read_acceleration(driver, imu_address)
        gyro_raw = read_gyro(driver, imu_address)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        dt = (time.monotonic_ns() - prev_time + offset) / 1e9
        ahrs.update_no_magnetometer(gyr, acc, dt)
        prev_time = time.monotonic_ns()

    print("✅ AHRS initialized. Capturing gravity-compensated bias...")

    bias = np.zeros(3)
    samples = 200
    for _ in range(samples):
        accel_raw = read_acceleration(driver, imu_address)
        if None in accel_raw.values():
            continue
        acc = convert_units(accel_raw)
        gravity_world = ahrs.quaternion.to_matrix().T @ grav_base
        bias_sample = acc - gravity_world.flatten()
        bias += bias_sample
        time.sleep(1 / sample_rate_hz)

    bias /= samples
    print(f"✅ IMU dynamic bias calibrated: ax={bias[0]:.3f}, ay={bias[1]:.3f}, az={bias[2]:.3f}")
    return bias, ahrs

### === Main Execution === ###

NetworkTables.initialize()
nt = NetworkTables.getTable("localization")

print("🔌 Connecting to DWM1001...")
dwm = dwm1001("/dev/ttyACM0")

print("🔌 Connecting to IMU...")
driver = HIDDriver()
imu_address = 0x69
initialize_icm20948(driver, imu_address)

bias, ahrs = calibrate_imu(driver, imu_address)

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

print("🚀 Starting sensor fusion loop...")
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
            accel_raw = read_acceleration(driver, imu_address)
            if None in accel_raw.values():
                print("⚠️ IMU read failed. Skipping this loop.")
                continue
            
            accel = convert_units(accel_raw) - bias  # body-frame acceleration
            roll, pitch, yaw = ahrs.quaternion.to_euler()

            # Only use forward (body Y) acceleration
            ay_body = accel[1]

            # Convert forward acceleration into world frame
            ax_world = ay_body * math.sin(yaw)
            ay_world = ay_body * math.cos(yaw)

            plain_ekf.dwm_update(anchors)
            fusion_ekf.dwm_update(anchors, ax_world, ay_world, accel[2])
            prev_imu = now

            print(f"[Fusion] ax={accel[0]:.2f} ay={accel[1]:.2f} az={accel[2]-1:.2f}")
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