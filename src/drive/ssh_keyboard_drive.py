import sys
import os
import tty
import termios
import select
import time
import math
import RPi.GPIO as GPIO
import numpy as np
from ahrs.filters import Madgwick

# Add AlphaBot import paths
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))
from alphabot.robot import AlphaBot2
from alphabot.imu_helper import HIDDriver, read_accel, read_gyro, convert_units, quaternion_to_yaw

# Initialize components
Ab = AlphaBot2()
Ab.setPWMA(22)
Ab.setPWMB(24.5)

d = HIDDriver()
madgwick = Madgwick()
imu_addr = 0x69
q = np.array([1.0, 0.0, 0.0, 0.0])

BUZ = 4
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZ, GPIO.OUT)

def beep_on():
    GPIO.output(BUZ, GPIO.HIGH)

def beep_off():
    GPIO.output(BUZ, GPIO.LOW)

def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

print("Control the AlphaBot using WASD keys. Press 'q' to quit.")
print("Press and hold keys for continuous movement.")

active_key = None
last_time = time.time()
last_yaw_time = time.time()
yaw = 0  # Integrated yaw

try:
    while True:
        key = get_key()
        # now = time.time()
        # dt = now - last_time
        # last_time = now

        # accel_raw = read_accel(d, imu_addr)
        # gyro_raw = read_gyro(d, imu_addr)
        # if None in accel_raw.values() or None in gyro_raw.values():
        #     continue
        # acc, gyr = convert_units(accel_raw, gyro_raw)
        # q = madgwick.updateIMU(q=q, gyr=gyr, acc=acc)
        # if q is not None:
        #     madgwick_yaw = quaternion_to_yaw(q) * 17.1

        # # Yaw from gyro integration
        # yaw += gyr[2] * dt
        # yaw = ((yaw + math.pi) % (2 * math.pi)) - math.pi

        if key is not None:
            if key == active_key:
                continue
            active_key = key

            if key == "w":
                Ab.forward()
            elif key == "a":
                Ab.left()
            elif key == "s":
                Ab.backward()
            elif key == "d":
                Ab.right()
            elif key == "q":
                print("Quitting...")
                break
        else:
            if active_key in ["w", "a", "s", "d"]:
                Ab.stop()
                print(f"Stopped movement for key: {active_key}")
            active_key = None

        # Print yaw every second
        # if now - last_yaw_time >= 0.2:
        #     print(f"Integrated Yaw: {math.degrees(yaw):.2f}°")
        #     print(f"Madgwick Yaw:  {math.degrees(madgwick_yaw):.2f}°")
        #     last_yaw_time = now

except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
