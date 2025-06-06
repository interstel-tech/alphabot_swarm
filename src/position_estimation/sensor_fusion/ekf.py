import serial
import time
import json
import redis
import math
import hid
import numpy as np
from ahrs.filters import Madgwick
from filterpy.kalman import ExtendedKalmanFilter

# ---------------- HID DRIVER ----------------
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

# ---------------- SENSOR HELPERS ----------------
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

def read_gyro_x(driver, addr):
    return read_word(driver, addr, 0x33, 0x34)

def convert_units(ax_raw, ay_raw, az_raw, gx_raw):
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    GRAVITY = 9.80665
    ax = (ax_raw / ACCEL_SCALE) * GRAVITY
    ay = (ay_raw / ACCEL_SCALE) * GRAVITY
    az = (az_raw / ACCEL_SCALE) * GRAVITY
    gx = math.radians(gx_raw / GYRO_SCALE + 1.1)
    return ax, ay, az, gx

# ---------------- EKF FUNCTIONS ----------------
def fx(x, dt):
    F = np.eye(4)
    F[0, 2] = dt
    F[1, 3] = dt
    return F @ x

def hx(x):
    return x[:2]

def HJacobian_at(x):
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

# ---------------- MAIN ----------------
if __name__ == "__main__":
    # Redis and Serial
    r = redis.Redis(host='localhost', port=6379, db=0)
    DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
    print("Connected to " + DWM.name)
    DWM.write("\r\r".encode())
    time.sleep(1)
    DWM.write("lec\r".encode())
    time.sleep(1)

    # IMU
    d = HIDDriver()
    imu_addr = 0x69
    d.write_byte_data(imu_addr, 0x06, 0x01)
    time.sleep(0.01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)
    time.sleep(0.01)

    madgwick = Madgwick()
    orientation = np.array([1.0, 0.0, 0.0, 0.0])

    # EKF setup
    ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
    ekf.x = np.array([0, 0, 0, 0])
    ekf.P *= 5
    ekf.R *= 0.05
    ekf.Q *= 0.01

    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # IMU read and filter
            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro_x(d, imu_addr)
            if accel_raw["x"] is None or gyro_raw is None:
                continue

            ax, ay, az, gx = convert_units(accel_raw["x"], accel_raw["y"], accel_raw["z"], gyro_raw)
            accel = np.array([ax, ay, az]) / 9.81
            gyro = np.array([0.0, gx, 0.0])
            orientation = madgwick.updateIMU(orientation, gyr=gyro, acc=accel)

            # Rotation matrix
            q = orientation
            R = np.array([
                [1 - 2*(q[2]**2 + q[3]**2),     2*(q[1]*q[2] - q[3]*q[0]),     2*(q[1]*q[3] + q[2]*q[0])],
                [2*(q[1]*q[2] + q[3]*q[0]),     1 - 2*(q[1]**2 + q[3]**2),     2*(q[2]*q[3] - q[1]*q[0])],
                [2*(q[1]*q[3] - q[2]*q[0]),     2*(q[2]*q[3] + q[1]*q[0]),     1 - 2*(q[1]**2 + q[2]**2)]
            ])
            world_accel = R @ (accel * 9.81) - np.array([0.0, 0.0, 9.81])

            # EKF Predict
            ekf.F = np.eye(4)
            ekf.F[0, 2] = dt
            ekf.F[1, 3] = dt
            ekf.predict()

            # UWB read
            data = DWM.readline().decode("utf-8").strip()
            if data and "POS" in data:
                data = data.replace("\r\n", "").split(",")
                try:
                    x = float(data[data.index("POS")+1])
                    y = float(data[data.index("POS")+2])
                    ekf.update(np.array([x, y]), HJacobian_at, hx)
                except Exception as e:
                    print(f"⚠️ Parsing error: {e}")

            print(f"📍 Fused x: {ekf.x[0]:.2f}, y: {ekf.x[1]:.2f}, θ: {math.degrees(gx):.2f}°")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
        DWM.write("\r".encode())
        DWM.close()
        d.h.close()
