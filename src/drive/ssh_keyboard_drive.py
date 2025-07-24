import sys
import os
import tty
import termios
import select
import RPi.GPIO as GPIO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))
from alphabot.robot import AlphaBot2

Ab = AlphaBot2()
Ab.setPWMA(25)
Ab.setPWMB(25)

BUZ = 4
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZ, GPIO.OUT)

def beep_on():
    GPIO.output(BUZ, GPIO.HIGH)

def beep_off():
    GPIO.output(BUZ, GPIO.LOW)

def get_key(timeout=0.1):
    """Reads a single keypress with optional timeout."""
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

try:
    while True:
        key = get_key()

        if key is not None:
            if key == active_key:
                continue  # ignore key repeat
            active_key = key

            if key == "w":
                Ab.forward()
                print("Moving forward")
            elif key == "a":
                Ab.left()
                print("Turning left")
            elif key == "s":
                Ab.backward()
                print("Moving backward")
            elif key == "d":
                Ab.right()
                print("Turning right")
            elif key == "q":
                print("Quitting...")
                break
        else:
            if active_key in ["w", "a", "s", "d"]:
                Ab.stop()
                print(f"Stopped movement for key: {active_key}")
            active_key = None

except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
