import time
import math
import hid
import numpy as np

# HID Driver to communicate with CP2112
class HIDDriver:
    def __init__(self,*,serial=None, led=True):
        h = self.h = hid.device()
        self.h.open(0x10C4, 0xEA90, serial)

        print("Manufacturer: %s" % h.get_manufacturer_string())
        print("Product: %s" % h.get_product_string())
        print("Serial No: %s" % h.get_serial_number_string())

        print('blink led set gpio')
        self.h.send_feature_report([0x03, 0xFF, 0x00, 0x00, 0x00])
        for _ in range(3):
            print('.')
            self.h.send_feature_report([0x04, 0x00, 0xFF])
            time.sleep(0.1)
            print('.')
            self.h.send_feature_report([0x04, 0xFF, 0xFF])
            time.sleep(0.1)

        self.gpio_direction = 0x83
        self.gpio_pushpull  = 0xFF
        self.gpio_special   = 0xFF
        self.gpio_clockdiv  = 1

        print("set gpio")
        self.h.send_feature_report([0x02, self.gpio_direction, self.gpio_pushpull, self.gpio_special, self.gpio_clockdiv])

        print("Set SMB Configuration (AN 495)")
        self.h.send_feature_report([0x06, 0x00, 0x01, 0x86, 0xA0, 0x02, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x01, 0x00, 0x0F])

    def read_byte_data(self, address, register):
        self.h.write([0x11, address << 1, 0x00, 0x01, 0x01, register])
        for _ in range(10):
            self.h.write([0x15, 0x01])
            response = self.h.read(7)
            if (response[0] == 0x16) and (response[2] == 5):
                self.h.write([0x12, 0x00, 0x01])
                response = self.h.read(4)
                return response[3]
        return None

    def write_byte_data(self, address, register, value):
        self.h.write([0x14, address << 1, 0x02, register, value])
        time.sleep(0.01)

# Initialize IMU
def initialize_icm20948(driver, addr):
    print("🔧 Initializing ICM-20948...")
    driver.write_byte_data(addr, 0x06, 0x01)
    time.sleep(0.01)
    driver.write_byte_data(addr, 0x3F, 0x00)
    time.sleep(0.01)
    print("✅ Initialization complete.")

# Read raw acceleration
def read_acceleration(driver, imu_address):
    registers = {
        "accel_x": (0x31, 0x32),
        "accel_y": (0x2F, 0x30),
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

# Read raw gyro x
def read_gyro_x(driver, imu_address):
    high = driver.read_byte_data(imu_address, 0x33)
    low = driver.read_byte_data(imu_address, 0x34)
    if high is None or low is None:
        return None
    raw = (high << 8) | low
    if raw & 0x8000:
        raw = -((65535 - raw) + 1)
    return raw

# Convert IMU readings to SI units
def convert_units(accel_raw, gyro_raw):
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    GRAVITY = 9.80665
    ax = (accel_raw["accel_x"] / ACCEL_SCALE) * GRAVITY
    ay = (accel_raw["accel_y"] / ACCEL_SCALE) * GRAVITY
    gx = gyro_raw / GYRO_SCALE
    return ax, ay, gx

# EKF Class
class EKF:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.x = np.zeros((5, 1))  # [x, y, vx, vy, theta]
        self.P = np.eye(5)
        self.Q = np.eye(5) * 0.05
        self.F = np.eye(5)
        self.B = np.zeros((5, 2))

    def predict(self, ax, ay, gx):
        theta = self.x[4, 0]
        dt = self.dt

        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.B[2, 0] = dt
        self.B[3, 1] = dt

        ax_w = ax * math.cos(theta) - ay * math.sin(theta)
        ay_w = ax * math.sin(theta) + ay * math.cos(theta)

        u = np.array([[ax_w], [ay_w]])
        self.x = self.F @ self.x + self.B @ u
        self.x[4, 0] += math.radians(gx * dt)
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x

if __name__ == "__main__":
    print("Start Estimation")
    d = HIDDriver()
    imu_address = 0x69
    initialize_icm20948(d, imu_address)

    ekf = EKF()
    last_time = time.time()

    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            ekf.dt = dt

            accel_raw = read_acceleration(d, imu_address)
            gyro_raw = read_gyro_x(d, imu_address)

            if accel_raw["accel_x"] is not None and gyro_raw is not None:
                ax, ay, gx = convert_units(accel_raw, gyro_raw)
                state = ekf.predict(ax, ay, gx)

                x, y, _, _, theta = state.flatten()
                print(f"Position Estimate (m): X: {x:.2f}, Y: {y:.2f}")
                print(f"Orientation Estimate (°): {math.degrees(theta):.2f}")
                print("-" * 60)
            else:
                print("⚠️ IMU data read failed.")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        d.h.close()
