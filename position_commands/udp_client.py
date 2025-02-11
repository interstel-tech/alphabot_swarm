import socket 
import RPi.GPIO as GPIO 
import time 
import math 
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from AlphaBot2 import AlphaBot2

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

# Robot's initial position
x_position = 0
y_position = 0

# Movement parameters
diameter = 4.2  # cm
circumference = diameter * math.pi  # cm
rps = 1.7  # revolutions per second (you might need to adjust this based on your robot's speed)
distance_per_second = circumference * rps  # cm per second

print("Waiting for movement commands...")

# Function to send periodic position updates
def send_position_update():
    global x_position, y_position
    position_message = f"{x_position},{y_position}"
    client_socket.sendto(position_message.encode(), (SERVER_IP, PORT))
    print(f"Position update sent: {position_message}")

# Time interval for reporting position (in seconds)
REPORT_INTERVAL = 0.5
last_report_time = time.time()

try:
    while True:
        # Receive data from the server
        data, addr = client_socket.recvfrom(1024)
        y, x = map(int, data.decode().split(","))  # Swapped x and y
        print(f"Received: x={x} cm, y={y} cm")
        
        # Calculate angle to turn
        angle = math.degrees(math.atan2(x, y))  # Swapped x and y in atan2
        print(f"Turning to {angle:.2f} degrees")
        
        if angle > 0:
            Ab.right()
        else:
            Ab.left()

        # Time needed to turn
        time.sleep(abs(angle) / 360)
        Ab.stop()

        # Move forward
        print("Moving forward")
        Ab.forward()
        distance = math.sqrt(x**2 + y**2)  # Euclidean distance to the target
        
        # Update robot's position
        total_distance_travelled = 0  # track how much distance has been covered
        time_to_move = distance / distance_per_second  # time required to move the full distance
        start_time = time.time()

        # Move in intervals, updating position periodically
        while total_distance_travelled < distance:
            # Move for a short time slice
            elapsed_time = time.time() - start_time
            total_distance_travelled = distance_per_second * elapsed_time

            # Update position based on movement
            x_position = total_distance_travelled * math.cos(math.radians(angle))
            y_position = total_distance_travelled * math.sin(math.radians(angle))

            # Periodically send position updates
            if time.time() - last_report_time >= REPORT_INTERVAL:
                send_position_update()
                last_report_time = time.time()

        Ab.stop()
        print("Movement complete")

except KeyboardInterrupt:
    print("Client shutting down.")
finally:
    client_socket.close()
    GPIO.cleanup()
    print("GPIO cleanup completed.")