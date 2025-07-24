import serial
import json
import redis
from alphabot.robot import AlphaBot2
import time
import math
from alphabot.imu_helper import HIDDriver, read_accel, read_gyro, convert_units, quaternion_to_yaw
from ahrs.filters import Madgwick
import numpy as np

Ab = AlphaBot2()
madgwick = Madgwick()
d = HIDDriver()
imu_addr = 0x69
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
r = redis.Redis(host='localhost', port=6379, db=0)

def cleanup():
    DWM.write(b"\r")
    DWM.close()

def get_position():
    while True:
        data = DWM.readline().decode("utf-8").strip()
        if "POS" in data:
            try:
                parts = data.split(",")
                current_x = float(parts[parts.index("POS") + 1])
                current_y = float(parts[parts.index("POS") + 2])
                pos_json = json.dumps({"x": current_x, "y": current_y})
                print("✅", pos_json)
                r.set("pos", pos_json)
                return current_x, current_y
            except Exception as e:
                print(f"[WARN] Bad POS line: {data}")
                continue

def set_position(x_target, y_target, yaw):
    print("Rotating to target...")
    madgwick = Madgwick()
    q = np.array([1.0, 0.0, 0.0, 0.0])
    last_time = time.time()
    d.write_byte_data(imu_addr, 0x06, 0x01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)
    Ab.setPWMA(25)
    Ab.setPWMB(25)

    last_time = time.time()
    x_pos, y_pos = get_position()

    target_angle = math.atan2(y_target - y_pos, x_target - x_pos) - math.radians(yaw)
    if target_angle > 0:
        Ab.left()
    else:
        Ab.right()
    initial_sign = math.copysign(1, yaw - target_angle)

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now
        accel_raw = read_accel(d, imu_addr)
        gyro_raw = read_gyro(d, imu_addr)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
        if q is not None:
            yaw = quaternion_to_yaw(q)
        yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7
        angle_error = math.degrees(yaw - target_angle)
        print(f"Yaw Error: {angle_error:.2f}")
        if abs(angle_error) < 5 or math.copysign(1, angle_error) != initial_sign:
            break
    Ab.stop()

    print("Moving to target...")

    # Initialize current position with starting estimate
    current_x = x_pos
    current_y = y_pos

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # IMU update
        accel_raw = read_accel(d, imu_addr)
        gyro_raw = read_gyro(d, imu_addr)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
        if q is not None:
            yaw = quaternion_to_yaw(q)
        yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7

        # UWB position update
        data = DWM.readline().decode("utf-8").strip()
        if "POS" in data:
            try:
                parts = data.split(",")
                current_x = float(parts[parts.index("POS") + 1])
                current_y = float(parts[parts.index("POS") + 2])
                pos_json = json.dumps({"x": current_x, "y": current_y})
                print("✅", pos_json)
                r.set("pos", pos_json)
            except:
                continue

            if abs(current_x - x_target) < 0.05 and abs(current_y - y_target) < 0.05:
                Ab.stop()
                break

        # Recalculate desired angle based on latest position
        dynamic_target_angle = math.atan2(y_target - current_y, x_target - current_x)

        # Yaw correction
        yaw_error = math.degrees(yaw - dynamic_target_angle)
        print(f"Yaw Drift: {yaw_error:.2f}")

        if yaw_error > 10:
            print("↩️ Correcting right")
            Ab.right()
        elif yaw_error < -10:
            print("↪️ Correcting left")
            Ab.left()
        else:
            Ab.forward()
    Ab.stop()
    print(math.degrees(yaw))
    return math.degrees(yaw)