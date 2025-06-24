import serial
import time

PORT = "/dev/ttyACM0"   # Change if your port is different
BAUDRATE = 115200

def send_command(ser, cmd, wait=0.2):
    ser.write((cmd + "\r").encode())
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        lines.append(line)
    return lines

def list_anchors(ser):
    print("\n📡 Anchor List:")
    output = send_command(ser, "lec")
    for line in output:
        print(line)

def list_nodes(ser):
    print("\n🧭 Network Nodes:")
    output = send_command(ser, "nls")
    for line in output:
        print(line)

def set_anchor_position(ser, anchor_id, x, y, z):
    cmd = f"aps {anchor_id} {x:.2f} {y:.2f} {z:.2f}"
    print(f"📌 Setting anchor {anchor_id} position to ({x}, {y}, {z})")
    response = send_command(ser, cmd)
    for line in response:
        print(line)

def save_network_config(ser):
    print("\n💾 Saving configuration...")
    output = send_command(ser, "nmg")
    for line in output:
        print(line)

def interactive_menu():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(1)  # Let serial settle
        ser.reset_input_buffer()

        print("Connected to UWB module.")
        while True:
            print("\n🔧 Menu:")
            print("1. List anchors")
            print("2. List all nodes")
            print("3. Set anchor position")
            print("4. Save config to flash")
            print("5. Exit")
            choice = input("Select option: ")

            if choice == "1":
                list_anchors(ser)
            elif choice == "2":
                list_nodes(ser)
            elif choice == "3":
                aid = input("Anchor ID (e.g. 1): ")
                x = float(input("X (m): "))
                y = float(input("Y (m): "))
                z = float(input("Z (m): "))
                set_anchor_position(ser, aid, x, y, z)
            elif choice == "4":
                save_network_config(ser)
            elif choice == "5":
                print("✅ Done.")
                break
            else:
                print("❌ Invalid option.")

if __name__ == "__main__":
    interactive_menu()