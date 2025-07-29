import serial
import json
import redis
import time
import math
import socket
import select
import numpy as np
import csv

from alphabot.robot import AlphaBot2
from alphabot.imu_helper import HIDDriver, read_accel, read_gyro, convert_units, quaternion_to_yaw
from ahrs.filters import Madgwick

Ab = AlphaBot2()
d = HIDDriver()
madgwick = Madgwick()
imu_addr = 0x69
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
# r = redis.Redis(host='localhost', port=6379, db=0)

def cleanup():
    DWM.write(b"\r")
    DWM.close()

def log_position_to_csv(x, y, filename="trajectory.csv"):
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([x, y])

def get_position():
    positions = []

    while len(positions) < 5:
        data = DWM.readline().decode("utf-8").strip()
        if "POS" in data:
            try:
                parts = data.split(",")
                current_x = float(parts[parts.index("POS") + 1])
                current_y = float(parts[parts.index("POS") + 2])
                positions.append((current_x, current_y))
                # print(f"✅ Reading {len(positions)}: x={current_x}, y={current_y}")
            except Exception as e:
                print(f"[WARN] Bad POS line: {data}")
                continue
        else:
            print(f"[WARN] No POS in line: {data}")
            continue

    # Compute average x and y
    avg_x = sum(p[0] for p in positions) / len(positions)
    avg_y = sum(p[1] for p in positions) / len(positions)
    pos_json = json.dumps({"x": avg_x, "y": avg_y})
    print("Average position:", pos_json)
    # r.set("pos", pos_json)
    return avg_x, avg_y


def set_position(x_target, y_target, yaw_offset, sock):
    q = np.array([1.0, 0.0, 0.0, 0.0])
    last_time = time.time()
    d.write_byte_data(imu_addr, 0x06, 0x01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)
    Ab.setPWMA(22)
    Ab.setPWMB(22)

    x_pos, y_pos = get_position()
    target_angle = math.atan2(y_target - y_pos, x_target - x_pos) - math.radians(yaw_offset)
    print(f"Target angle: {math.degrees(target_angle)}°")

    print("Rotating to target...")

    Ab.left() if target_angle > 0 else Ab.right()
    initial_sign = math.copysign(1, math.radians(yaw_offset) - target_angle)

    while True:
        # Non-blocking check for UDP command
        if select.select([sock], [], [], 0)[0]:
            data1, _ = sock.recvfrom(1024)
            message = json.loads(data1.decode('utf-8'))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (x_target, y_target):
                    print(f"❌ Interrupted! New target: {new_x}, {new_y}")
                    Ab.stop()
                    return math.degrees(yaw)

        # IMU update
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

    current_x, current_y = x_pos, y_pos

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Non-blocking check for UDP interrupt
        if select.select([sock], [], [], 0)[0]:
            data1, _ = sock.recvfrom(1024)
            message = json.loads(data1.decode('utf-8'))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (x_target, y_target):
                    print(f"❌ Interrupted! New target: {new_x}, {new_y}")
                    Ab.stop()
                    return math.degrees(yaw)

        # IMU update
        accel_raw = read_accel(d, imu_addr)
        gyro_raw = read_gyro(d, imu_addr)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
        if q is not None:
            yaw = quaternion_to_yaw(q)
        yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7 + math.radians(yaw_offset)

        current_x, current_y = get_position()

        # Log current position to CSV
        log_position_to_csv(current_x, current_y)

        if abs(current_x - x_target) < 0.1 and abs(current_y - y_target) < 0.1:
            Ab.stop()
            break

        # Yaw correction
        dynamic_target_angle = math.atan2(y_target - current_y, x_target - current_x)
        yaw_error = math.degrees(yaw - dynamic_target_angle)
        # print(f"Yaw: {math.degrees(yaw):.2f}")
        # print(f"Target Angle: {math.degrees(dynamic_target_angle):.2f}")
        print(f"Yaw Drift: {yaw_error:.2f}")

        if yaw_error > 10:
            print("↩️ Correcting right")
            Ab.right()
            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)
            if None in accel_raw.values() or None in gyro_raw.values():
                continue
            acc, gyr = convert_units(accel_raw, gyro_raw)
            q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
            if q is not None:
                yaw = quaternion_to_yaw(q)
            yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7 + math.radians(yaw_offset)
            time.sleep(0.1)
            Ab.stop()
        elif yaw_error < -10:
            print("↪️ Correcting left")
            Ab.left()
            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)
            if None in accel_raw.values() or None in gyro_raw.values():
                continue
            acc, gyr = convert_units(accel_raw, gyro_raw)
            q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
            if q is not None:
                yaw = quaternion_to_yaw(q)
            yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7 + math.radians(yaw_offset)
            time.sleep(0.1)
            Ab.stop()
        else:
            Ab.forward()

        time.sleep(0.01)

    Ab.stop()
    print(math.degrees(yaw))
    return math.degrees(yaw)

def get_position():
    positions = []

    while len(positions) < 5:
        data = DWM.readline().decode("utf-8").strip()
        if "POS" in data:
            try:
                parts = data.split(",")
                current_x = float(parts[parts.index("POS") + 1])
                current_y = float(parts[parts.index("POS") + 2])
                positions.append((current_x, current_y))
                # print(f"✅ Reading {len(positions)}: x={current_x}, y={current_y}")
            except Exception as e:
                print(f"[WARN] Bad POS line: {data}")
                continue
        else:
            print(f"[WARN] No POS in line: {data}")
            continue

    # Compute average x and y
    avg_x = sum(p[0] for p in positions) / len(positions)
    avg_y = sum(p[1] for p in positions) / len(positions)
    pos_json = json.dumps({"x": avg_x, "y": avg_y})
    print("Average position:", pos_json)
    # r.set("pos", pos_json)
    return avg_x, avg_y


def set_vector(x_vector, y_vector, yaw_offset, sock):
    q = np.array([1.0, 0.0, 0.0, 0.0])
    last_time = time.time()
    d.write_byte_data(imu_addr, 0x06, 0x01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)
    Ab.setPWMA(22)
    Ab.setPWMB(24.5)

    x_pos, y_pos = get_position()
    target_angle = math.atan2(y_vector, x_vector) - math.radians(yaw_offset)
    print(f"Target angle: {math.degrees(target_angle)}°")

    print("Rotating to target...")

    Ab.left() if target_angle > 0 else Ab.right()
    initial_sign = math.copysign(1, math.radians(yaw_offset) - target_angle)

    while True:
        # Non-blocking check for UDP command
        if select.select([sock], [], [], 0)[0]:
            data1, _ = sock.recvfrom(1024)
            message = json.loads(data1.decode('utf-8'))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (x_vector, y_vector):
                    print(f"❌ Interrupted! New vector: {new_x}, {new_y}")
                    Ab.stop()
                    return math.degrees(yaw)

        # IMU update
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

    current_x, current_y = x_pos, y_pos

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Non-blocking check for UDP interrupt
        if select.select([sock], [], [], 0)[0]:
            data1, _ = sock.recvfrom(1024)
            message = json.loads(data1.decode('utf-8'))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (x_vector, y_vector):
                    print(f"❌ Interrupted! New target: {new_x}, {new_y}")
                    Ab.stop()
                    return math.degrees(yaw)

        # IMU update
        accel_raw = read_accel(d, imu_addr)
        gyro_raw = read_gyro(d, imu_addr)
        if None in accel_raw.values() or None in gyro_raw.values():
            continue
        acc, gyr = convert_units(accel_raw, gyro_raw)
        q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
        if q is not None:
            yaw = quaternion_to_yaw(q)
        yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7 + math.radians(yaw_offset)

        # Log current position to CSV
        # log_position_to_csv(current_x, current_y)

        # Yaw correction
        yaw_error = math.degrees(target_angle - yaw)
        print(f"Yaw Drift: {yaw_error:.2f}")

        if yaw_error > 10:
            print("↩️ Correcting right")
            Ab.right()
            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)
            if None in accel_raw.values() or None in gyro_raw.values():
                continue
            acc, gyr = convert_units(accel_raw, gyro_raw)
            q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
            if q is not None:
                yaw = quaternion_to_yaw(q)
            yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7 + math.radians(yaw_offset)
            time.sleep(0.1)
            Ab.stop()
        elif yaw_error < -10:
            print("↪️ Correcting left")
            Ab.left()
            accel_raw = read_accel(d, imu_addr)
            gyro_raw = read_gyro(d, imu_addr)
            if None in accel_raw.values() or None in gyro_raw.values():
                continue
            acc, gyr = convert_units(accel_raw, gyro_raw)
            q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
            if q is not None:
                yaw = quaternion_to_yaw(q)
            yaw = ((yaw + math.pi) % (2 * math.pi) - math.pi) * 9.7 + math.radians(yaw_offset)
            time.sleep(0.1)
            Ab.stop()
        else:
            Ab.forward()

        time.sleep(0.01)

    Ab.stop()
    print(math.degrees(yaw))
    return math.degrees(yaw)