import serial
import json
import redis
import time
import math
import socket
import select
import numpy as np

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


# def get_position():
#     while True:
#         data = DWM.readline().decode("utf-8").strip()
#         if "POS" in data:
#             try:
#                 parts = data.split(",")
#                 current_x = float(parts[parts.index("POS") + 1])
#                 current_y = float(parts[parts.index("POS") + 2])
#                 pos_json = json.dumps({"x": current_x, "y": current_y})
#                 print("✅", pos_json)
#                 # r.set("pos", pos_json)
#                 return current_x, current_y
#             except Exception as e:
#                 print(f"[WARN] Bad POS line: {data}")
#                 continue
#         else:
#             print(f"[WARN] No POS in line: {data}")
#             continue

def normalize360(angle: float) -> float:
    return (angle + 360) % 360

def get_position(num_samples=1):
    """Get averaged position and continuously update yaw."""
    positions = []

    while len(positions) < num_samples:
        data = DWM.readline().decode("utf-8").strip()
        # Always update yaw even if POS data isn't received

        if "POS" in data:
            try:
                parts = data.split(",")
                current_x = float(parts[parts.index("POS") + 1])
                current_y = float(parts[parts.index("POS") + 2])
                positions.append((current_x, current_y))
            except Exception as e:
                print(f"[WARN] Bad POS line: {data}")
                continue
        else:
            print(f"[WARN] No POS in line: {data}")

    avg_x = sum(p[0] for p in positions) / len(positions)
    avg_y = sum(p[1] for p in positions) / len(positions)

    pos_json = json.dumps({"x": avg_x, "y": avg_y})

    return avg_x, avg_y

def set_position(x_target, y_target, yaw_offset, sock):
    q = np.array([1.0, 0.0, 0.0, 0.0])
    last_time = time.time()
    d.write_byte_data(imu_addr, 0x06, 0x01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)
    Ab.setPWMA(22)
    Ab.setPWMB(24.5)

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

def signed_error(yaw, target):
    error = (target - yaw + 180) % 360 - 180
    return error

    
def set_vector(x_vector, y_vector, yaw_offset, sock):
    yaw = 0.0
    last_time = time.time()

    d.write_byte_data(imu_addr, 0x06, 0x01)
    d.write_byte_data(imu_addr, 0x3F, 0x00)
    Ab.setPWMA(22)
    Ab.setPWMB(24)

    current_x, current_y = get_position()

    # --- Compute target angle ---
    target_angle = math.degrees(math.atan2(y_vector, x_vector)) - yaw_offset
    target_angle = normalize360(target_angle)
    print(f"Target angle: {target_angle:.2f}°")

    # --- Rotate to target ---
    print("Rotating to target...")
    Ab.left()  # always CCW

    # Save addr from first UDP command so we can reply
    addr = None  

    while True:
        # Non-blocking UDP check
        if select.select([sock], [], [], 0)[0]:
            data1, addr = sock.recvfrom(1024)
            message = json.loads(data1.decode("utf-8"))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (x_vector, y_vector):
                    print(f"❌ Interrupted! New target: {new_x}, {new_y}")
                    Ab.stop()
                    return yaw

        # IMU integration
        now = time.time()
        dt = now - last_time
        last_time = now

        gyro_raw = read_gyro(d, imu_addr)
        if None in gyro_raw.values():
            continue
        _, gyr = convert_units({"x": 0, "y": 0, "z": 0}, gyro_raw)
        gz = gyr[2]

        yaw += math.degrees(gz * dt)
        yaw = normalize360(yaw)

        print(f"Yaw: {yaw:.2f} | Target: {target_angle:.2f}")

        # --- Send yaw update back ---
        # if addr:
        #     pos_json = json.dumps({"x": current_x, "y": current_y, "yaw": yaw})
        #     sock.sendto(pos_json.encode("utf-8"), addr)

        if yaw >= target_angle - 2:
            break

    Ab.stop()
    print("✅ Rotation complete. Moving to target...")

    # --- Move forward with drift correction ---
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Non-blocking UDP check
        if select.select([sock], [], [], 0)[0]:
            data1, addr = sock.recvfrom(1024)
            message = json.loads(data1.decode("utf-8"))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (x_vector, y_vector):
                    print(f"❌ Interrupted! New target: {new_x}, {new_y}")
                    Ab.stop()
                    return yaw

        # IMU integration
        gyro_raw = read_gyro(d, imu_addr)
        if None in gyro_raw.values():
            continue
        _, gyr = convert_units({"x": 0, "y": 0, "z": 0}, gyro_raw)
        gz = gyr[2]

        current_x, current_y = get_position()

        yaw += math.degrees(gz * dt)
        yaw = normalize360(yaw)

        # signed drift correction
        yaw_error = signed_error(yaw, target_angle)
        print(f"Yaw: {yaw:.2f} | Drift: {yaw_error:.2f}")

        if yaw_error > 10:
            print("↩️ Correcting left")
            Ab.left()
            time.sleep(0.05)
            # time.sleep(0.1)
            # Ab.stop()
        elif yaw_error < -10:
            print("↪️ Correcting right")
            Ab.right()
            time.sleep(0.05)
            # time.sleep(0.1)
            # Ab.stop()
        else:
            Ab.forward()

        # --- Send continuous updates ---
        # if addr:
        #     pos_json = json.dumps({"x": current_x, "y": current_y, "yaw": yaw})
        #     sock.sendto(pos_json.encode("utf-8"), addr)

        time.sleep(0.01)
