import socket
import json
import time
import RPi.GPIO as GPIO
from alphabot.position import set_position, cleanup

def RunAlphabotController():
    # Setup UDP socket
    udp_ip = "0.0.0.0"
    udp_port = 50001
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    print(f"Listening for JSON commands on UDP port {udp_port}...")

    first_command = True
    yaw = 0.0

    try:
        while True:
            data, _ = sock.recvfrom(1024)  # buffer size
            try:
                message = json.loads(data.decode('utf-8'))
                a_col = message.get("s", {}).get("col", None)
                if not a_col or len(a_col) < 2:
                    print("Invalid 's.col' format.")
                    continue

                x_target, y_target = float(a_col[0]), float(a_col[1])
                print(f"Received target: x = {x_target}, y = {y_target}")

                if first_command:
                    yaw = set_position(x_target, y_target, 0.0)
                    first_command = False
                else:
                    yaw = set_position(x_target, y_target, yaw)

                print(f"Updated yaw: {yaw}")

            except json.JSONDecodeError:
                print("Received invalid JSON.")
            except Exception as e:
                print(f"Error handling data: {e}")

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        sock.close()
        cleanup()

if __name__ == "__main__":
    RunAlphabotController()
