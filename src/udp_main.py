import socket
import json
import time
import redis
import RPi.GPIO as GPIO
from alphabot.position import set_position, cleanup

def RunAlphabotController():
    r = redis.Redis(host='localhost', port=6379, db=0)
    udp_ip = "0.0.0.0"
    udp_port = 50001
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    print(f"Listening for JSON commands on UDP port {udp_port}...")

    yaw = 0.0
    last_target = (None, None)  # Store last x, y to compare

    try:
        while True:
            data, _ = sock.recvfrom(1024)
            try:
                message = json.loads(data.decode('utf-8'))
                a_col = message.get("s", {}).get("col", None)
                if not a_col or len(a_col) < 2:
                    print("Invalid 's.col' format.")
                    continue

                x_target, y_target = float(a_col[0]), float(a_col[1])
                current_target = (x_target, y_target)

                # Skip if target hasn't changed
                if current_target == last_target:
                    print(f"🔁 Duplicate target ({x_target}, {y_target}) — skipping.")
                    continue

                print(f"🛰️ Received new target: x = {x_target}, y = {y_target}")
                r.set("target", json.dumps({"x": x_target, "y": y_target}))
                yaw = set_position(x_target, y_target, yaw)
                last_target = current_target
                print(f"✅ Updated yaw: {yaw}")

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
