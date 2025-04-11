import time
import hid
import numpy as np
import serial
import json
import redis

# Redis setup
r = redis.Redis(host='localhost', port=6379, db=0)

# HID Driver for CP2112
class HIDDriver:
    def __init__(self,*,serial=None, led=True):
        h = self.h = hid.device()
        self.h.open(0x10C4, 0xEA90, serial)
        print("Manufacturer:", h.get_manufacturer_string())
        print("Product:", h.get_product_string())
        print("Serial No:", h.get_serial_number_string())

        self.h.send_feature_report([0x03, 0xFF, 0x00, 0x00, 0x00])
        for _ in range(3):
            self.h.send_feature_report([0x04, 0x00, 0xFF])
            time.sleep(0.1)
            self.h.send_feature_report([0x04, 0xFF, 0xFF])
            time.sleep(0.1)

        self.gpio_direction = 0x83
        self.gpio_pushpull  = 0xFF
        self.gpio_special   = 0xFF
        self.gpio_clockdiv  = 1
        self.h.send_feature_report([0x02, self.gpio_direction, self.gpio_pushpull, self.gpio_special, self.gpio_clockdiv])
        self.h.send_feature_report([0x06, 0x00, 0x01, 0x86, 0xA0, 0x02, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x01, 0x00, 0x0F])

    def read_byte_data(self, address, register):
        self.h.write([0x11, address << 1, 0x00, 0x01, 0x01, register])
        for _ in range(10):
            self.h.write([0x15, 0x01])
            response = self.h.read(7)
            if response[0] == 0x16 and response[2] == 5:
                self.h.write([0x12, 0x00, 0x01])
                return self.h.read(4)[3]
        return None

    def write_byte_data(self, address, register, value):
        self.h.write([0x14, address << 1, 0x02, register, value])
        time.sleep(0.01)

# IMU setup
def initialize_icm20948(driver, addr):
    print("🔧 Initializing ICM-20948...")
    driver.write_byte_data(addr, 0x06, 0x01)  # PWR_MGMT_1
    time.sleep(0.01)
    driver.write_byte_data(addr, 0x3F, 0x00)  # ACCEL_CONFIG
    time.sleep(0.01)
    print("✅ Initialization complete.")

def read_acceleration(driver, imu_address):
    regs = {
        "x": (0x31, 0x32),  # ax
        "y": (0x2F, 0x30),  # ay
    }
    acc = {}
    for axis, (high_reg, low_reg) in regs.items():
        high = driver.read_byte_data(imu_address, high_reg)
        low  = driver.read_byte_data(imu_address, low_reg)
        if high is None or low is None:
            return None
        raw = (high << 8) | low
        if raw & 0x8000:
            raw = -((65535 - raw) + 1)
        acc[axis] = raw
    return acc

def convert_acceleration(accel_raw):
    GRAVITY = 9.80665
    SCALE = 16384.0
    return {
        "x": (accel_raw["x"] / SCALE) * GRAVITY,
        "y": (accel_raw["y"] / SCALE) * GRAVITY
    }

# EKF
class ExtendedKalmanFilter:
    def __init__(self, f, h, F, H, Q, R, x0):
        self.f = f
        self.h = h
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = np.eye(len(x0))

    def predict(self, u):
        self.x = self.f(self.x, u)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.h(self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

# System model
dt = 0.1
def f(x, u):
    return np.array([
        x[0] + x[1]*dt + 0.5*u[0]*dt**2,
        x[1] + u[0]*dt,
        x[2] + x[3]*dt + 0.5*u[1]*dt**2,
        x[3] + u[1]*dt
    ])

def h(x):
    return np.array([x[0], x[2]])

F = np.array([
    [1, dt, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, dt],
    [0, 0, 0, 1]
])
H = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0]
])
Q = np.diag([0.1, 0.1, 0.1, 0.1])
R = np.diag([0.3, 0.3])
x0 = np.array([0, 0, 0, 0])
kf = ExtendedKalmanFilter(f, h, F, H, Q, R, x0)

# UWB Setup
uwb = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
uwb.write("\r\r".encode())
time.sleep(1)
uwb.write("lec\r".encode())
time.sleep(1)

# Main loop
driver = HIDDriver()
imu_address = 0x69
initialize_icm20948(driver, imu_address)

try:
    while True:
        accel_raw = read_acceleration(driver, imu_address)
        if accel_raw is None:
            print("⚠️ IMU read error.")
            continue

        acc = convert_acceleration(accel_raw)
        kf.predict(np.array([acc["x"], acc["y"]]))

        line = uwb.readline().decode("utf-8").strip()
        if "POS" in line:
            try:
                parts = line.split(",")
                x = float(parts[parts.index("POS") + 1])
                y = float(parts[parts.index("POS") + 2])
                kf.update(np.array([x, y]))

                est = kf.x
                print(f"UWB: x={x:.2f}, y={y:.2f} | EKF Pos: x={est[0]:.2f}, y={est[2]:.2f}")
                r.set("pos", json.dumps({"x": round(est[0], 2), "y": round(est[2], 2)}))
            except Exception as e:
                print("⚠️ UWB parse error:", e)

        time.sleep(dt)

except KeyboardInterrupt:
    print("Stopping...")
    uwb.write("\r".encode())
    uwb.close()
    driver.h.close()
