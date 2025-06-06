import time
import hid
import math

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

        self.gpio_direction = 0x00
        self.gpio_pushpull  = 0x00
        self.gpio_special   = 0x00
        self.gpio_clockdiv  = 1

        if led:
            self.gpio_direction = 0x83
            self.gpio_pushpull  = 0xFF
            self.gpio_special   = 0xFF

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
        self.h.write([0x14, address << 1, 0x02, register, value])  # SMBus Write with Command Code
        time.sleep(0.01)

# Function to initialize the IMU 
def initialize_icm20948(driver, addr):
    print("🔧 Initializing ICM-20948...")

    # USER BANK 0
    driver.write_byte_data(addr, 0x06, 0x01)  # PWR_MGMT_1: clear sleep bit
    time.sleep(0.01)

    driver.write_byte_data(addr, 0x3F, 0x00)  # ACCEL_CONFIG: ±2g (default)
    time.sleep(0.01)

    print("✅ Initialization complete.")

# Function to read acceleration data from IMU
def read_acceleration(driver, imu_address):
    # ACCEL_XOUT_H = 0x2D
    registers = {
        "accel_x": (0x2D, 0x2E),
        "accel_y": (0x2F, 0x30),
        "accel_z": (0x31, 0x32)
    }

    accel_data = {}

    for axis, (high_reg, low_reg) in registers.items():
        high = driver.read_byte_data(imu_address, high_reg)
        low = driver.read_byte_data(imu_address, low_reg)

        if high is None or low is None:
            print(f"⚠️ Could not read {axis}")
            accel_data[axis] = None
            continue

        raw = (high << 8) | low

        # Convert to signed 16-bit integer
        if raw & 0x8000:
            raw = -((65535 - raw) + 1)

        accel_data[axis] = raw

    return accel_data

# Function to read gyro data from IMU
def read_gyroscope(driver, imu_address):
    registers = {
        "gyro_x": (0x33, 0x34),
        "gyro_y": (0x35, 0x36),
        "gyro_z": (0x37, 0x38)
    }

    gyro_data = {}

    for axis, (high_reg, low_reg) in registers.items():
        high = driver.read_byte_data(imu_address, high_reg)
        low = driver.read_byte_data(imu_address, low_reg)

        if high is None or low is None:
            print(f"⚠️ Could not read {axis}")
            gyro_data[axis] = None
            continue

        raw = (high << 8) | low
        if raw & 0x8000:
            raw = -((65535 - raw) + 1)

        gyro_data[axis] = raw

    return gyro_data

# Converts raw IMU data to physical units (m/s² and °/s)
def convert_imu_data(accel_raw, gyro_raw):
    ACCEL_SCALE_MODIFIER = 16384.0  # ±2g full scale
    GYRO_SCALE_MODIFIER = 131.0     # ±250 °/s full scale
    GRAVITY = 9.80665               # m/s² per g

    ax = (accel_raw['accel_x'] / ACCEL_SCALE_MODIFIER) * GRAVITY
    ay = (accel_raw['accel_y'] / ACCEL_SCALE_MODIFIER) * GRAVITY
    az = (accel_raw['accel_z'] / ACCEL_SCALE_MODIFIER) * GRAVITY

    gx = gyro_raw['gyro_x'] / GYRO_SCALE_MODIFIER + 1.1
    gy = gyro_raw['gyro_y'] / GYRO_SCALE_MODIFIER
    gz = gyro_raw['gyro_z'] / GYRO_SCALE_MODIFIER

    return {
        "accel_mps2": {"x": ax, "y": ay, "z": az},
        "gyro_dps": {"x": gx, "y": gy, "z": gz}
    }

if __name__ == "__main__":
    print("Start Acceleration Readout")
    d = HIDDriver()
    imu_address = 0x69

    initialize_icm20948(d, imu_address)

    yaw = 0.0  # initial yaw angle in radians
    last_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            accel_raw = read_acceleration(d, imu_address)
            gyro_raw = read_gyroscope(d, imu_address)

            if accel_raw["accel_x"] is not None and gyro_raw["gyro_z"] is not None:
                converted = convert_imu_data(accel_raw, gyro_raw)

                a = converted["accel_mps2"]
                g = converted["gyro_dps"]

                # Integrate X-axis angular velocity (convert °/s to rad/s)
                yaw += (g["x"] * math.pi / 180.0) * dt
                yaw_deg = math.degrees(yaw)

                print(f"X Accel (m/s²): {a['x']:.2f}")
                print(f"Y Accel (m/s²): {a['y']:.2f}")
                print(f"Yaw Angle (°): {yaw_deg:.2f}")
                print("-" * 40)

            else:
                print("⚠️ Failed to read full sensor data.")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        d.h.close()
