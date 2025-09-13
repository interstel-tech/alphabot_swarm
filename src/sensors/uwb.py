import serial
import time
import json

def connect_uwb(DWM: serial.Serial, return_after_stable: bool = True):
    uwb_mode = "lep"  # Options: "lep", "loc", "pos"
    # Resend mode command if no data for a while
    missed = 0
    # Count of consecutive good reads, return after threshold
    good = 0
    # Initialize UWB module
    DWM.write(b"\r\r")
    time.sleep(1)
    DWM.write(f"{uwb_mode}\r".encode("utf-8"))
    time.sleep(1)
    while True:
        print("🔄 Waiting for data...")
        try:
            data = DWM.readline().decode("utf-8").strip()
        except UnicodeDecodeError as e:
            print("⚠️ Decode error:", e)
            continue

        if not data:
            print("⚠️ No data received.")
            missed += 1
            good = 0
        elif "POS" in data:
            try:
                parts = data.split(",")
                x = parts[parts.index("POS")+1]
                y = parts[parts.index("POS")+2]
                pos = {"x": x, "y": y}
                pos_json = json.dumps(pos)
                print("✅", pos_json)
                # r.set("pos", pos_json)
                missed = 0
                good += 1
                if good > 10 and return_after_stable:
                    print("ℹ️ UWB seems stable now.")
                    return
            except Exception as e:
                print("⚠️ Parse error:", e)
                missed += 1
                good = 0
        else:
            print("⚠️ Non-position data:", data)
            missed += 1
            good = 0

        # Auto-recover if UWB goes silent
        if missed > 5:
            print("🔁 Re-sending lep command to recover...")
            DWM.write(b"\r\r")
            time.sleep(1)
            DWM.write(f"{uwb_mode}\r".encode("utf-8"))
            time.sleep(1)
            missed = 0

if __name__ == "__main__":
    DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
    print("Connected to " + DWM.name)
    try:
        connect_uwb(DWM, return_after_stable=False)
    except KeyboardInterrupt:
        print("🛑 Stopping...")
        DWM.write(b"\r")
        DWM.close()