import socket

# Set up the UDP server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('192.168.33.249', 12000))  # Listen on all interfaces on port 12000

print("Server is listening for direction durations...")

try:
    while True:
        # Prompt user for duration values (x: horizontal, y: vertical)
        x = input("Enter duration for x (positive for right, negative for left in s): ")
        y = input("Enter duration for y (positive for forward, negative for backward in s): ")

        # Convert input to a message
        message = f"{y},{x}".encode()  # Encode durations as "x,y"

        # Send to the robot client
        robot_address = ('192.168.33.222', 12000) 
        server_socket.sendto(message, robot_address)
        print(f"Sent command: x={x} ms, y={y} ms")
except KeyboardInterrupt:
    print("Server shutting down.")
finally:
    server_socket.close()