import time
import json
import numpy as np
import qwiic_icm20948
import serial
import redis

# Initialize Redis
r = redis.Redis(host='localhost', port=6379, db=0)

# Connect to UWB
DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " + DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)

# Connect to IMU
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print("IMU not connected!")
    exit(1)
IMU.begin()

# Constants
ACCEL_SCALE_MODIFIER = 16384.0  # For ±2g
GRAVITY = 9.80665               # m/s² per g
dt = 0.1  # 10 Hz loop

# EKF setup
class ExtendedKalmanFilter:
    def __init__(self, f, h, F, H, Q, R, P=None, x0=None):
        self.f = f
        self.h = h
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.P = np.eye(len(x0)) if P is None else P
        self.x = np.zeros_like(x0) if x0 is None else x0

    def predict(self, u):
        self.x = self.f(self.x, u)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.h(self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

# System models
def f(x, u):
    return np.array([
        x[0] + x[1]*dt + 0.5*u[0]*dt**2,  # pos_x
        x[1] + u[0]*dt,                  # vel_x
        x[2] + x[3]*dt + 0.5*u[1]*dt**2,  # pos_y
        x[3] + u[1]*dt                   # vel_y
    ])

def h(x):
    return np.array([x[0], x[2]])  # Measuring pos_x and pos_y

F = np.array([
    [1, dt, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, dt],
    [0, 0, 0, 1]
])

H = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0]
])

Q = np.diag([0.1, 0.1, 0.1, 0.1])  # Process noise
R = np.diag([0.3, 0.3])            # UWB measurement noise
x0 = np.array([0, 0, 0, 0])        # Initial state: [pos_x, vel_x, pos_y, vel_y]

kf = ExtendedKalmanFilter(f=f, h=h, F=F, H=H, Q=Q, R=R, x0=x0)

# Main loop
try:
    while True:
        if not IMU.dataReady():
            time.sleep(0.05)
            continue

        IMU.getAgmt()
        ax = (IMU.axRaw / ACCEL_SCALE_MODIFIER) * GRAVITY
        ay = (IMU.ayRaw / ACCEL_SCALE_MODIFIER) * GRAVITY

        # Predict with accelerometer as input
        accel_input = np.array([ax, ay])
        kf.predict(accel_input)

        # Read UWB
        data = DWM.readline().decode("utf-8").strip()
        if "POS" in data:
            tokens = data.replace("\r\n", "").split(",")
            try:
                raw_x = float(tokens[tokens.index("POS")+1])
                raw_y = float(tokens[tokens.index("POS")+2])

                kf.update(np.array([raw_x, raw_y]))

                est_x, est_y = kf.x[0], kf.x[2]
                print(f"UWB: x={raw_x:.2f}, y={raw_y:.2f} | EKF: x={est_x:.2f}, y={est_y:.2f}")

                pos = {"x": round(est_x, 3), "y": round(est_y, 3)}
                r.set("pos", json.dumps(pos))

            except Exception as e:
                print("Parsing error:", e)

        time.sleep(dt)

except KeyboardInterrupt:
    print("Stopping...")
    DWM.write("\r".encode())
    DWM.close()
