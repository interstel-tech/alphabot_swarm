import math
import hid
import time
import numpy as np
from ahrs.filters import Madgwick

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
    gz = math.radians(gyro_raw["z"] / GYRO_SCALE) + 0.018
    return np.array([ax, ay, az]), np.array([gx, gy, gz])

def quaternion_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

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
