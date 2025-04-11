import time
import math
import numpy as np
import hid

# CP2112 HID driver
class HIDDriver:
    def __init__(self, *, serial=None):
        self.h = hid.device()
        self.h.open(0x10C4, 0xEA90, serial)

        print("Manufacturer: %s" % self.h.get_manufacturer_string())
        print("Product: %s" % self.h.get_product_string())
        print("Serial No: %s" % self.h.get_serial_number_string())

        print("Set GPIO")
        self.h.send_feature_report([0x03, 0xFF, 0x00, 0x00, 0x00])
        self.h.send_feature_report([0x04, 0x00, 0xFF])
        self.h.send_feature_report([0x04, 0xFF, 0xFF])

        self.h.send_feature_report([0x02, 0x83, 0xFF, 0xFF, 0x01])
        print("Set SMBus config")
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

# IMU functions
def initialize_icm20948(driver, addr):
    driver.write_byte_data(addr, 0x06, 0x01)
    time.sleep(0.01)
    driver.write_byte_data(addr, 0x3F, 0x00)
    time.sleep(0.01)
    print("✅ IMU initialized")

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
    gx = gx_raw / GYRO_SCALE
    return ax, ay, az, gx

def compensate_gravity(ax, ay, az):
    # Compute roll and pitch
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    # Build rotation matrix from body to world frame
    R = np.array([
        [math.cos(pitch), math.sin(pitch) * math.sin(roll), math.sin(pitch) * math.cos(roll)],
        [0,               math.cos(roll),                  -math.sin(roll)],
        [-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll)]
    ])

    a_body = np.array([[ax], [ay], [az]])
    a_world = R.T @ a_body  # Transform to world frame

    ax_world = a_world[0, 0]
    ay_world = a_world[1, 0]
    return ax_world, ay_world, math.degrees(pitch), math.degrees(roll)

# Main
if __name__ == "__main__":
    d = HIDDriver()
    imu_addr = 0x69
    initialize_icm20948(d, imu_addr)

    velocity = {"x": 0.0, "y": 0.0}
    position = {"x": 0.0, "y": 0.0}
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
                ax, ay, az, gx = convert_units(accel_raw["x"], accel_raw["y"], accel_raw["z"], gyro_raw)
                ax_corr, ay_corr, pitch, roll = compensate_gravity(ax, ay, az)

                # Integrate gyroscope
                angle_x += gx * dt

                # Integrate acceleration to velocity
                velocity["x"] += ax_corr * dt
                velocity["y"] += ay_corr * dt

                # Integrate velocity to position
                position["x"] += velocity["x"] * dt
                position["y"] += velocity["y"] * dt

                print(f"Accel (m/s²): X_raw: {ax:.2f}, Y_raw: {ay:.2f}, Z: {az:.2f}")
                print(f"Accel (m/s²) Corrected: X: {ax_corr:.2f}, Y: {ay_corr:.2f}")
                print(f"Gyro  (°/s):  X: {gx:.2f}")
                print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
                print(f"Position Estimate (m): X: {position['x']:.2f}, Y: {position['y']:.2f}")
                print(f"Orientation Estimate (°): {angle_x:.2f}")
                print("-" * 60)
            else:
                print("⚠️ Failed to read sensor data")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting...")
        d.h.close()
