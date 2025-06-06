import time
import math
import serial
import redis
import hid
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

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
        "z": read_word(driver, addr, 0x33, 0x34)  # <-- Z-axis used for yaw
    }

def convert_units(accel_raw, gyro_raw):
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0  # deg/s
    GRAVITY = 9.80665
    ax = (accel_raw["x"] / ACCEL_SCALE) * GRAVITY
    ay = (accel_raw["y"] / ACCEL_SCALE) * GRAVITY
    az = (accel_raw["z"] / ACCEL_SCALE) * GRAVITY
    gx = math.radians(gyro_raw["x"] / GYRO_SCALE) 
    gy = math.radians(gyro_raw["y"] / GYRO_SCALE)
    gz = math.radians(gyro_raw["z"] / GYRO_SCALE + 1.1) 
    return np.array([ax, ay, az]), np.array([gx, gy, gz])

def hx(x): return x[:2]
def HJacobian_at(x): return np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

if __name__ == "__main__":
    r = redis.Redis(host='localhost', port=6379, db=0)
    DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
    print("Connected to " + DWM.name)
    DWM.write(b"\r\r")
    time.sleep(1)
    DWM.write(b"lec\r")
    time.sleep(1)

    d = HIDDriver()
    imu_addr = 0x69
    d.write_byte_data(imu_addr, 0x06, 0x01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)

    ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
    ekf.x = np.array([0.0, 0.0, 0.0, 0.0])
    ekf.P *= 5
    ekf.R *= 0.1
    ekf.Q *= 0.01

    yaw = 0.0
    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)

            if None in accel_raw.values() or None in gyro_raw.values():
                continue

            acc, gyr = convert_units(accel_raw, gyro_raw)

            # Integrate Z gyro (yaw)
            yaw += gyr[2] * dt
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
            yaw_deg = math.degrees(yaw)

            ekf.F = np.eye(4)
            ekf.F[0, 2] = dt
            ekf.F[1, 3] = dt
            ekf.predict()

            data = DWM.readline().decode("utf-8").strip()
            if "POS" in data:
                try:
                    parts = data.split(",")
                    x = float(parts[parts.index("POS")+1])
                    y = float(parts[parts.index("POS")+2])
                    ekf.update(np.array([x, y]), HJacobian_at, hx)
                except Exception as e:
                    print("⚠️ UWB Parse Error:", e)

            print(f"Fused x: {ekf.x[0]:.2f}, y: {ekf.x[1]:.2f}, θ (yaw): {yaw_deg:.2f}°")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping...")
        DWM.write(b"\r")
        DWM.close()
        d.h.close()
