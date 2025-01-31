import RPi.GPIO as GPIO
from sshkeyboard import listen_keyboard, stop_listening  # Added stop_listening for proper cleanup
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from AlphaBot2 import AlphaBot2

Ab = AlphaBot2()

Ab.setPWMA(20) # Set motor speed to 20%
Ab.setPWMB(20) # Set motor speed to 20%

BUZ = 4

def beep_on():
    GPIO.output(BUZ, GPIO.HIGH)

def beep_off():
    GPIO.output(BUZ, GPIO.LOW)

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BUZ, GPIO.OUT)

# Function to handle keypress events
def press_key(key):
    if key == "w":  # Forward
        Ab.forward()
        print("Moving forward")
    elif key == "a":  # Left
        Ab.left()
        print("Turning left")
    elif key == "s":  # Backward
        Ab.backward()
        print("Moving backward")
    elif key == "d":  # Right
        Ab.right()
        print("Turning right")
    elif key == "q":  # Quit
        print("Exiting...")
        stop_listening()  # Stop the keyboard listener gracefully

# Function to handle key release events
def release_key(key):
    if key in ["w", "a", "s", "d"]:  # Only stop for movement keys
        Ab.stop()
        print(f"Stopped movement for key: {key}")

print("Control the AlphaBot using WASD keys. Press 'Q' to quit.")
print("Listening for key events...")

try:
    # Start listening for keypress and release events
    listen_keyboard(on_press=press_key, on_release=release_key)
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    GPIO.cleanup()  # Ensure GPIO pins are reset
    print("GPIO cleanup completed.")
