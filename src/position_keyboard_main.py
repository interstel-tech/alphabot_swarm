from alphabot.position import set_position, cleanup
import time
import serial
import RPi.GPIO as GPIO

def RunAlphabotController():
    # --- Setup ---
    # DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
    # print("Connected to " + DWM.name)
    # DWM.write(b"\r\r")
    # time.sleep(1)
    # DWM.write(b"lec\r")
    # time.sleep(1)

    try:
        while True:
            first_command = True
            user_input = input("Enter target x y (or q to quit): ")
            if user_input.strip().lower() == 'q':
                break

            try:
                x_target, y_target = map(float, user_input.strip().split())
            except:
                print("Invalid input. Enter x y or q.")
                continue

            # print("Collecting initial UWB readings for averaging...")
            # x_list, y_list = [], []
            # while len(x_list) < 20:
            #     data = DWM.readline().decode("utf-8").strip()
            #     if "POS" in data:
            #         try:
            #             parts = data.split(",")
            #             x = float(parts[parts.index("POS") + 1])
            #             y = float(parts[parts.index("POS") + 2])
            #             x_list.append(x)
            #             y_list.append(y)
            #             print(f"Reading {len(x_list)}/20: x={x:.2f}, y={y:.2f}")
            #         except:
            #             continue

            # x_pos = sum(x_list) / len(x_list)
            # y_pos = sum(y_list) / len(y_list)
            # print(f"📍 Averaged starting position: x={x_pos:.2f}, y={y_pos:.2f}")

            if first_command:
                yaw = set_position(x_target, y_target, 0.0)
                print(yaw)
                first_command = False
            else:   
                yaw = set_position(x_target, y_target, yaw)
                print(yaw)


    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        # DWM.write(b"\r")
        # DWM.close()
        # GPIO.cleanup()
        cleanup()


if __name__ == "__main__":
    RunAlphabotController()