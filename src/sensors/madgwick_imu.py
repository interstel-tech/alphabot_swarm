import time
import math
import hid
import numpy as np
from ahrs.filters import Madgwick

# CP2112 HID driver
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

# Sensor utilities
def initialize_icm20948(driver, addr):
    driver.write_byte_data(addr, 0x06, 0x01)
    time.sleep(0.01)
    driver.write_byte_data(addr, 0x3F, 0x00)
    time.sleep(0.01)

def read_word(driver, addr, high_reg, low_reg):
    high = driver.read_byte_data(addr, high_reg)
    low = driver.read_byte_data(addr, low_reg)
    if high is None or low is None:
        return None
    raw = (high << 8) | low
    if raw & 0x8000:
        raw = -((65535 - raw) + 1)
    return raw

def read_accel(driver, addr):
    ax = read_word(driver, addr, 0x31, 0x32)
    ay = read_word(driver, addr, 0x2F, 0x30)
    az = read_word(driver, addr, 0x2D, 0x2E)
    return {"x": ax, "y": ay, "z": az}

def read_gyro_x(driver, addr):
    return read_word(driver, addr, 0x33, 0x34)

def convert_units(ax_raw, ay_raw, az_raw, gx_raw):
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    GRAVITY = 9.80665

    ax = (ax_raw / ACCEL_SCALE) * GRAVITY
    ay = (ay_raw / ACCEL_SCALE) * GRAVITY
    az = (az_raw / ACCEL_SCALE) * GRAVITY
    gx = math.radians(gx_raw / GYRO_SCALE)
    return ax, ay, az, gx

def quaternion_to_euler(q):
    w, x, y, z = q
    # roll (x-axis)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    # pitch (y-axis)
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    # yaw (z-axis)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw

# Main
if __name__ == "__main__":
    d = HIDDriver()
    imu_addr = 0x69
    initialize_icm20948(d, imu_addr)

    madgwick = Madgwick()
    orientation = np.array([1.0, 0.0, 0.0, 0.0])
    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro_x(d, imu_addr)

            if accel_raw["x"] is not None and gyro_raw is not None:
                ax, ay, az, gx = convert_units(
                    accel_raw["x"], accel_raw["y"], accel_raw["z"], gyro_raw
                )
                accel = np.array([ax, ay, az]) / 9.81
                gyro = np.array([0.0, gx, 0.0])

                orientation = madgwick.updateIMU(orientation, gyr=gyro, acc=accel)
                roll, pitch, yaw = quaternion_to_euler(orientation)

                print(f"Roll: {math.degrees(roll):.2f}° | "
                      f"Pitch: {math.degrees(pitch):.2f}° | "
                      f"Yaw: {math.degrees(yaw):.2f}°")
                print("-" * 50)

            time.sleep(0.01)

    except KeyboardInterrupt:
        d.h.close()
