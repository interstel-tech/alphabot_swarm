import serial
import json
import time
import math
import socket
import select
import numpy as np
import threading

from alphabot.robot import AlphaBot2
from alphabot.imu_helper import HIDDriver, read_accel, read_gyro, convert_units, quaternion_to_yaw
from ahrs.filters import Madgwick

PWMA_SPEED = 22
PWMB_SPEED = 24.5

# Ab = AlphaBot2()
# d = HIDDriver()
# madgwick = Madgwick()
# imu_addr = 0x69
# DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
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

class PositionSamples:
    """
    Thread-safe position samples storage.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self.samples = []
    def add(self, v):
        print(f"Adding position sample: {v}, total samples: {len(self.samples)+1}")
        with self._lock:
            self.samples.append(v)
    def clear(self):
        with self._lock:
            self.samples = []
    def get_average(self):
        with self._lock:
            print(f"Calculating average of {len(self.samples)} samples")
            if not self.samples:
                return (0.0, 0.0)
            return (sum(p[0] for p in self.samples) / len(self.samples), sum(p[1] for p in self.samples) / len(self.samples))

class AlphaBotDriver():
    def __init__(self):
        self.Ab = AlphaBot2()
        self.d = HIDDriver()
        self.DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
        self.imu_addr = 0x69
        self.sock = None
        self.addr = None
        self.positions = PositionSamples()
        self.last_pos = (0.0, 0.0)
        self.yaw = 0.0
        self.target_pos = (0.0, 0.0)
        self.target_vel = (0.0, 0.0)
        self.is_running: threading.Event = threading.Event()
        self.perform_sampling: threading.Event = threading.Event()
        self.uwb_is_ready: threading.Event = threading.Event()
        self.sampling_thread = threading.Thread(target=self.position_sampler, daemon=True)
        # Set initial flags
        self.is_running.set()
        self.uwb_is_ready.set()
        # Initialize IMU and motors
        self.d.write_byte_data(self.imu_addr, 0x06, 0x01)
        self.d.write_byte_data(self.imu_addr, 0x3F, 0x00)
        # Initialize Ultra Wide Band
        self.DWM.write(b"\r\r")
        time.sleep(0.5)
        self.DWM.write(b"lep\r")
        time.sleep(0.5)
        # Start position sampler thread and get initial position
        self.sampling_thread.start()
        self.perform_sampling.set()
        time.sleep(2.0)
        self.perform_sampling.clear()
        self.last_pos = self.positions.get_average()
        print(f"Initial position: {self.last_pos}")

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        self.is_running.clear()
        self.Ab.stop()
        self.perform_sampling.clear()
        self.sampling_thread.join(timeout=1.0)
        if (self.DWM.is_open):
            self.DWM.write(b"\r")
            self.DWM.close()

    def udp_listen(self):
        # Non-blocking UDP check
        if select.select([self.sock], [], [], 0)[0]:
            data1, addr = self.sock.recvfrom(1024)
            message = json.loads(data1.decode("utf-8"))
            new_col = message.get("s", {}).get("col", None)
            if new_col and len(new_col) >= 2:
                new_x, new_y = float(new_col[0]), float(new_col[1])
                if (new_x, new_y) != (self.target_vel[0], self.target_vel[1]):
                    print(f"❌ Interrupted! New target: {new_x}, {new_y}")
                    self.Ab.stop()
                    return True
        return False
    
    def position_sampler(self):
        """
        Continuously samples position via UWB.
        There appears to be a buffer on the UWB that can cause
        a lag of ~30 samples if not read frequently enough,
        so this thread runs continuously and stores samples
        for estimation when enabled via the perform_sampling event.
        """
        failure_count = 0
        while self.is_running.is_set() and self.uwb_is_ready.is_set():
            data = self.DWM.readline().decode("utf-8").strip()
            if "POS" in data:
                try:
                    parts = data.split(",")
                    current_x = float(parts[parts.index("POS") + 1])
                    current_y = float(parts[parts.index("POS") + 2])
                    if self.perform_sampling.is_set():
                        self.positions.add((current_x, current_y))
                    else:
                        print(f"Sampled: {current_x}, {current_y}")
                except Exception as e:
                    print(f"[WARN] Bad POS line: {data}, {e}")
                    continue
            else:
                print(f"[WARN] No POS in line: {data}")
                failure_count += 1
                if failure_count > 5:
                    print(f"Exiting program. Please reinitialize UWB module.")
                    self.uwb_is_ready.clear()

    def imu_update(self, dt):
        # IMU integration
        gyro_raw = read_gyro(self.d, self.imu_addr)
        if None in gyro_raw.values():
            return
        _, gyr = convert_units({"x": 0, "y": 0, "z": 0}, gyro_raw)
        gz = gyr[2]

        self.yaw += math.degrees(gz * dt)
        self.yaw = normalize360(self.yaw)

    def correct_orientation(self):
        """
        Rotate to face target direction.
        """
        # Reducing speed to improve turning accuracy and to not overshoot
        self.Ab.setPWMA(PWMA_SPEED * 0.75)
        self.Ab.setPWMB(PWMB_SPEED * 0.75)

        # --- Compute target angle ---
        target_angle = math.degrees(math.atan2(self.target_vel[1], self.target_vel[0]))
        # Flag for printing
        turning_left = None
        # Used for calculating dt for IMU integration
        last_loop_time = time.time()

        while True:
            # Non-blocking UDP check
            if self.udp_listen():
                self.Ab.stop()
                return False
            
            yaw_error = signed_error(self.yaw, target_angle)
            print(f"Yaw: {self.yaw:.2f} | Yaw Error: {yaw_error:.2f}")

            if abs(yaw_error) < 5:
                break

            if yaw_error > 10 and (not turning_left or turning_left is None):
                print("↩️ Correcting left")
                self.Ab.left()
                turning_left = True
            elif yaw_error < -10 and (turning_left or turning_left is None):
                print("↪️ Correcting right")
                self.Ab.right()
                turning_left = False

            # IMU integration
            current_time = time.time()
            self.imu_update(current_time - last_loop_time)
            last_loop_time = current_time

            # --- Send yaw update back ---
            # if addr:
            #     pos_json = json.dumps({"x": current_x, "y": current_y, "yaw": yaw})
            #     sock.sendto(pos_json.encode("utf-8"), addr)
            time.sleep(0.01)

        self.Ab.stop()
        print("✅ Rotation complete. Moving to target...")
        return True

    def move_forward(self, duration=1.0):
        self.Ab.setPWMA(PWMA_SPEED)
        self.Ab.setPWMB(PWMB_SPEED)
        # Used to calculate total duration for exit condition of the loop
        duration_start_time = time.time()
        # Used for calculating dt for IMU integration
        last_loop_time = time.time()
        # --- Move forward with drift tracking ---
        self.Ab.forward()
        while time.time() - duration_start_time < duration:
            # IMU integration
            current_time = time.time()
            self.imu_update(current_time - last_loop_time)
            last_loop_time = current_time
            time.sleep(0.01)
        self.Ab.stop()

    def estimate_position_and_yaw(self, duration=1.0):
        # --- Estimate position over duration ---
        time.sleep(duration)
        current_pos = self.positions.get_average()
        # Get an estimate of the yaw and average it with the IMU yaw
        est_yaw = math.degrees(math.atan2(current_pos[1] - self.last_pos[1], current_pos[0] - self.last_pos[0]))
        est_yaw = normalize360(est_yaw)
        print(f"Estimated Position: {current_pos}, Estimated Yaw: {est_yaw:.2f}°, IMU Yaw: {self.yaw:.2f}°")
        self.yaw = (est_yaw + self.yaw) / 2
        self.yaw = normalize360(self.yaw)

    def set_vector(self, x_vector, y_vector, yaw_offset, sock):
        self.target_vel = (x_vector, y_vector)
        self.yaw = yaw_offset
        self.sock = sock
        self.addr = None
        #
        while self.is_running.is_set():
            while self.uwb_is_ready.is_set():
                # Turn to target direction
                if not self.correct_orientation():
                    return self.yaw
                # Lock in current position as last known
                self.last_pos = self.positions.get_average()
                self.perform_sampling.clear()
                self.positions.clear()
                # Move forward a bit
                self.move_forward(duration=2.0)
                # Start position sampling and estimate new position
                self.perform_sampling.set()
                self.estimate_position_and_yaw(duration=0.75)
            # TODO: Handle UWB failure (e.g., reinitialize)
            raise RuntimeError("UWB module failure detected. Please reinitialize UWB module")
        self.Ab.stop()
        self.perform_sampling.clear()

# Note to self:
# make sure to clear position list afterwards
# Add mutex to position list access if needed
