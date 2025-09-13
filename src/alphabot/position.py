import serial
import json
import time
import math
import socket
import select
import numpy as np
import threading
from enum import Enum
from typing import Tuple

from alphabot.robot import AlphaBot2
from alphabot.imu_helper import HIDDriver, read_accel, read_gyro, convert_units, quaternion_to_yaw
from ahrs.filters import Madgwick
from sensors.uwb import connect_uwb

PWMA_SPEED = 22
PWMB_SPEED = 25
DISTANCE_TOLERANCE = 0.25  # meters

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
    def atomic_clear_and_add(self, v):
        with self._lock:
            self.samples = [v]
    def clear(self):
        with self._lock:
            self.samples = []
    def num_samples(self):
        with self._lock:
            return len(self.samples)
    def get_average(self):
        with self._lock:
            if not self.samples:
                raise ValueError("No position samples available")
            print(f"Calculating average of {len(self.samples)} samples")
            return (sum(p[0] for p in self.samples) / len(self.samples), sum(p[1] for p in self.samples) / len(self.samples))

class DriveMode(Enum):
    POSITION = 1
    VECTOR = 2

def is_within_tolerance(a: Tuple[float, float], b: Tuple[float, float], tol: float) -> bool:
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

def mid_angle(a: float, b: float) -> float:
    """Calculate the mid-angle between two angles in degrees."""
    a_rad = math.radians(a)
    b_rad = math.radians(b)
    x = (math.cos(a_rad) + math.cos(b_rad)) / 2
    y = (math.sin(a_rad) + math.sin(b_rad)) / 2
    mid_rad = math.atan2(y, x)
    return math.degrees(mid_rad) % 360

class AlphaBotDriver():
    def __init__(self):
        self.Ab = AlphaBot2()
        self.d = HIDDriver()
        self.DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
        self.imu_addr = 0x69
        self.sock = None
        self.addr = None
        self.drive_mode = DriveMode.POSITION
        self.positions = PositionSamples()
        self.last_pos = (0.0, 0.0)
        self.yaw = 0.0
        self.target_pos: Tuple[float, float] | None = None
        self.target_vel: Tuple[float, float] = (0.0, 0.0)
        self.is_running: threading.Event = threading.Event()
        self.perform_sampling: threading.Event = threading.Event()
        self.uwb_is_ready: threading.Event = threading.Event()
        self.sampling_thread = threading.Thread(target=self.position_sampler, daemon=True)
        # Initialize IMU and motors
        self.d.write_byte_data(self.imu_addr, 0x06, 0x01)
        self.d.write_byte_data(self.imu_addr, 0x3F, 0x00)
        # Initialize Ultra Wide Band
        connect_uwb(self.DWM, return_after_stable=True)
        # Set initial flags
        self.uwb_is_ready.set()
        self.is_running.set()
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

    def did_get_new_target(self):
        """
        Poll the UDP socket for new target commands.
        """
        # Non-blocking UDP check
        if select.select([self.sock], [], [], 0)[0]:
            data1, addr = self.sock.recvfrom(1024)
            message = json.loads(data1.decode("utf-8"))
            new_pos = message.get("s", {}).get("col", None)
            new_vel = message.get("v", {}).get("col", None)
            # Prioritizes position updates over velocity updates
            if new_pos and len(new_pos) >= 2:
                new_x, new_y = float(new_pos[0]), float(new_pos[1])
                if self.drive_mode == DriveMode.VECTOR or (self.drive_mode == DriveMode.POSITION and (new_x, new_y) != (self.target_pos[0], self.target_pos[1])):
                    self.target_pos = (new_x, new_y)
                    print(f"❌ Interrupted! New POSITION target: {new_x}, {new_y}")
                    self.Ab.stop()
                    return True
            elif new_vel and len(new_vel) >= 2:
                new_x, new_y = float(new_vel[0]), float(new_vel[1])
                if self.drive_mode == DriveMode.POSITION or (self.drive_mode == DriveMode.VECTOR and (new_x, new_y) != (self.target_vel[0], self.target_vel[1])):
                    self.target_vel = (new_x, new_y)
                    print(f"❌ Interrupted! New VELOCITY target: {new_x}, {new_y}")
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
                        # Only keep one sample when not sampling
                        self.positions.atomic_clear_and_add((current_x, current_y))
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
        self.Ab.setPWMA(PWMA_SPEED * 0.9)
        self.Ab.setPWMB(PWMB_SPEED * 0.9)

        first_time = True

        while True:
            # Non-blocking UDP check
            if first_time or self.did_get_new_target():
                self.Ab.stop()
                # --- Compute target angle ---
                if self.drive_mode == DriveMode.POSITION and self.target_pos:
                    current_pos = self.positions.get_average()
                    self.target_vel = (self.target_pos[0] - current_pos[0], self.target_pos[1] - current_pos[1])
                # TODO: Handle zero velocity case
                target_angle = math.degrees(math.atan2(self.target_vel[1], self.target_vel[0]))
                turning_left = None
                last_loop_time = time.time()
                first_time = False
            
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

            time.sleep(0.01)

        self.Ab.stop()
        print("✅ Rotation complete. Moving to target...")

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
            # Non-blocking UDP check
            if self.did_get_new_target():
                break
            # IMU integration
            current_time = time.time()
            self.imu_update(current_time - last_loop_time)
            last_loop_time = current_time
            time.sleep(0.01)
        self.Ab.stop()

    def estimate_position_and_yaw(self, duration=1.0):
        """
        Estimate new position and yaw after moving forward.
        Uses the collected position samples to compute an average position,
        then estimates yaw based on movement direction and averages it with IMU yaw.
        """
        # --- Estimate position over duration ---
        time.sleep(duration)
        current_pos = self.positions.get_average()
        # Get an estimate of the yaw and average it with the IMU yaw
        est_yaw = math.degrees(math.atan2(current_pos[1] - self.last_pos[1], current_pos[0] - self.last_pos[0]))
        est_yaw = normalize360(est_yaw)

        print(f"Estimated Position: {current_pos}, Estimated Yaw: {est_yaw:.2f}°, IMU Yaw: {self.yaw:.2f}°")
        self.yaw = mid_angle(self.yaw, est_yaw)

    def run_drive_loop(self, drive_mode: DriveMode, x: float, y: float, sock):
        """
        Run the drive loop for the specified drive mode and target position/velocity.
        This function will block until the target is reached (for POSITION mode)
        or until interrupted by a new target (for both modes).
        Parameters:
        - drive_mode: DriveMode.POSITION or DriveMode.VECTOR
        - x, y: Target position (for POSITION mode) or velocity vector (for VECTOR mode)
        - sock: UDP socket for receiving new target commands
        Returns:
        - None
        """
        self.drive_mode = drive_mode
        if drive_mode == DriveMode.POSITION:
            self.target_pos = (x, y)
        elif drive_mode == DriveMode.VECTOR:
            self.target_vel = (x, y)
        self.sock = sock
        self.addr = None

        # Ensure we have some initial position samples
        if self.positions.num_samples() < 3:
            self.perform_sampling.set()
            time.sleep(0.4)

        while self.is_running.is_set():
            while self.uwb_is_ready.is_set():
                # --- Exit Conditions ---
                # Check if we reached the target (for POSITION mode)
                if (self.drive_mode == DriveMode.POSITION and is_within_tolerance(self.positions.get_average(), self.target_pos, DISTANCE_TOLERANCE)):
                    print(f"✅ Reached target position: {self.target_pos}")
                    self.Ab.stop()
                    self.perform_sampling.clear()
                    return

                # --- Main Drive Logic ---
                # Turn to target direction
                self.correct_orientation()
                # Lock in current position as last known
                self.last_pos = self.positions.get_average()
                self.perform_sampling.clear()
                self.positions.clear()
                # Send position update back to controller
                pos_json = json.dumps({"x": self.last_pos[0], "y": self.last_pos[1]})
                if self.sock and self.addr:
                    self.sock.sendto(pos_json.encode('utf-8'), self.addr)
                # Move forward a bit
                # TODO: Handle overshooting caused by constant velocity and duration
                self.move_forward(duration=2.0)
                # Resume position sampling and estimate new position
                self.perform_sampling.set()
                self.estimate_position_and_yaw(duration=0.33)
            # UWB lost, try to reconnect
            print("🔁 UWB lost, attempting to reconnect...")            
            connect_uwb(self.DWM, return_after_stable=True)
        self.Ab.stop()
        self.perform_sampling.clear()
        return

