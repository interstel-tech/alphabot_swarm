import socket
import json
from alphabot.position import AlphaBotDriver, DriveMode

import faulthandler
faulthandler.enable()

def RunAlphabotController():
    # r = redis.Redis(host='localhost', port=6379, db=0)
    udp_ip = "0.0.0.0"
    udp_port = 50001
    udp_response_port = 50002
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    dest_ip = None
    print(f"Listening for JSON commands on UDP port {udp_port}...")
    print(f"Sending responses on UDP port {udp_response_port}...")

    yaw = 0.0
    last_target = (None, None)  # Store last x, y to compare

    alphabot_driver = AlphaBotDriver()

    try:
        while True:
            data, addr = sock.recvfrom(1024)
            try:
                message = json.loads(data.decode('utf-8'))
                print(f"Received message: {message}")
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
                # r.set("target", json.dumps({"x": x_target, "y": y_target}))
                alphabot_driver.run_drive_loop(DriveMode.POSITION, x_target, y_target, sock, (addr[0], udp_response_port))
                last_target = current_target

            except json.JSONDecodeError:
                print("Received invalid JSON.")
            except Exception as e:
                print(f"Caught exception: {e}")
                break

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        alphabot_driver.cleanup()
        sock.close()

if __name__ == "__main__":
    RunAlphabotController()