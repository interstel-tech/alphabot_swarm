import json
import redis
import time
import math
import hid
import numpy as np

# EKF for 2D position estimation with UWB correction
class EKF2D:
    def __init__(self):
        self.x = np.zeros((4, 1))  # [x, y, vx, vy]
        self.P = np.eye(4) * 0.1
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1])
        self.R = np.diag([0.5, 0.5])
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

    def predict(self, ax, ay, dt):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        B = np.array([[0.5 * dt**2, 0],
                      [0, 0.5 * dt**2],
                      [dt, 0],
                      [0, dt]])
        u = np.array([[ax], [ay]])
        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

    def update(self, meas_x, meas_y):
        z = np.array([[meas_x], [meas_y]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

    def get_state(self):
        return self.x.flatten()

# HIDDriver for CP2112
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
    gx = gx_raw / GYRO_SCALE + 1.1
    return ax, ay, az, gx

# Main fusion loop
def run_fusion():
    r = redis.Redis(host='localhost', port=6379, db=0)
    d = HIDDriver()
    imu_addr = 0x69
    initialize_icm20948(d, imu_addr)
    ekf = EKF2D()
    angle_x = 0.0
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

                # EKF predict step
                ekf.predict(ax, ay, dt)

                # Gyro-based orientation estimate
                angle_x += gx * dt

                # EKF update step (if new UWB available)
                pos_json = r.get("pos")
                if pos_json:
                    try:
                        pos = json.loads(pos_json)
                        uwb_x = float(pos["x"])
                        uwb_y = float(pos["y"])
                        ekf.update(uwb_x, uwb_y)
                    except Exception:
                        pass

                est = ekf.get_state()
                print(f"Accel (m/s²): X: {ax:.2f}, Y: {ay:.2f}")
                print(f"Gyro  (°/s):  X: {gx:.2f}")
                print(f"Position EKF (m): X: {est[0]:.2f}, Y: {est[1]:.2f}")
                print(f"Orientation (°): {angle_x:.2f}")
                print("-" * 60)

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Exiting...")
        d.h.close()

run_fusion()
