import socket
import threading

# Set up the UDP server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('192.168.33.249', 12000))  # Listen on port 12000

print("Server is listening for commands and position updates...")

# Function to receive and print position updates continuously
def receive_positions():
    while True:
        try:
            data, addr = server_socket.recvfrom(1024)  # Receive position update
            print(f"Received position update from {addr}: {data.decode()}")
        except Exception as e:
            print(f"Error receiving position update: {e}")

# Start the position receiving thread
receiver_thread = threading.Thread(target=receive_positions, daemon=True)
receiver_thread.start()

try:
    while True:
        # Prompt user for movement commands
        x = input("Enter movement in x (cm, positive for right, negative for left): ")
        y = input("Enter movement in y (cm, positive for forward, negative for backward): ")

        # Convert input to a message
        message = f"{y},{x}".encode()  # Encode as "y,x"

        # Send to the robot client
        robot_address = ('192.168.33.222', 12000)
        server_socket.sendto(message, robot_address)
        print(f"Sent command: x={x} cm, y={y} cm")

except KeyboardInterrupt:
    print("Server shutting down.")
finally:
    server_socket.close()