import socket
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
import time
import math

# Initialize AlphaBot2
Ab = AlphaBot2()
Ab.setPWMA(20)  # Set motor speed to 20%
Ab.setPWMB(20)  # Set motor speed to 20%

# UDP Client Configuration
CLIENT_IP = "192.168.33.222"
SERVER_IP = "192.168.33.249"
PORT = 12000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind((CLIENT_IP, PORT))  # Bind to receive messages

print("Waiting for movement commands...")

try:
    while True:
        # Receive data from the server
        data, addr = client_socket.recvfrom(1024)
        x, y = map(int, data.decode().split(","))
        print(f"Received: x={x} ms, y={y} ms")
        
        # Calculate angle to turn
        angle = math.degrees(math.atan2(y, x))  
        print(f"Turning to {angle:.2f} degrees")
        
        if angle > 0:
            Ab.right()
        else:
            Ab.left()

        # Calculating time needed to turn
        time.sleep(abs(angle) / 180)  
        Ab.stop()
        
        # Move forward
        print("Moving forward")
        Ab.forward()
        time.sleep(math.sqrt(x**2 + y**2))  
        Ab.stop()
        print("Movement complete")

except KeyboardInterrupt:
    print("Client shutting down.")
finally:
    client_socket.close()
    GPIO.cleanup()
    print("GPIO cleanup completed.")
