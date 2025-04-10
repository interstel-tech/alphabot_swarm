import serial
import time
import json
import redis
import numpy as np

# Initialize Redis
r = redis.Redis(host='localhost', port=6379, db=0)

# Initialize Serial Connection
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " + DWM.name)

# Send Commands to Start Data Stream
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)

import numpy as np

# Kalman filter class
class KalmanFilter(object):
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None):
        if F is None or H is None:
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.m) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


# Kalman Filter Initialization
dt = 1.0 / 10  # 10 Hz update rate
F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H = np.array([1, 0, 0]).reshape(1, 3)
Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R = np.array([0.5]).reshape(1, 1)

kf_x = KalmanFilter(F=F, H=H, Q=Q, R=R)
kf_y = KalmanFilter(F=F, H=H, Q=Q, R=R)


try:
    while True:
        # Read and decode data
        data = DWM.readline().decode("utf-8").strip()

        if data and "POS" in data:
            data = data.replace("\r\n", "").split(",")
            try:
                raw_x = float(data[data.index("POS")+1])
                raw_y = float(data[data.index("POS")+2])

                kf_x.predict()
                kf_y.predict()

                kf_x.update(np.array([[raw_x]]))
                kf_y.update(np.array([[raw_y]]))

                filtered_x = float(np.dot(H, kf_x.x)[0])
                filtered_y = float(np.dot(H, kf_y.x)[0])


                pos = {"x": round(filtered_x, 3), "y": round(filtered_y, 3)}
                pos_json = json.dumps(pos)

                print(f"Raw: x={raw_x:.2f}, y={raw_y:.2f} | Filtered: x={filtered_x:.2f}, y={filtered_y:.2f}")
                r.set("pos", pos_json)

            except (ValueError, IndexError) as e:
                print(f"Error parsing: {e}")

        time.sleep(0.1)  # Update every 0.1 seconds

except KeyboardInterrupt:
    print("Stopping...")
    DWM.write("\r".encode())
    DWM.close()
