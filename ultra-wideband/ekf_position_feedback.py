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

# Extended Kalman filter class
class ExtendedKalmanFilter(object):
    def __init__(self, f, h, F, H, Q=None, R=None, P=None, x0=None):
        # f: non-linear state transition function
        # h: non-linear measurement function
        # F: Jacobian of the state transition function
        # H: Jacobian of the measurement function
        if f is None or h is None or F is None or H is None:
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.f = f  # State transition function
        self.h = h  # Measurement function
        self.F = F  # Jacobian of the state transition function
        self.H = H  # Jacobian of the measurement function

        self.Q = np.eye(self.n) if Q is None else Q  # Process noise covariance
        self.R = np.eye(self.m) if R is None else R  # Measurement noise covariance
        self.P = np.eye(self.n) if P is None else P  # Estimation error covariance
        self.x = np.zeros((self.n, 1)) if x0 is None else x0  # Initial state estimate

    def predict(self, u=0):
        # Use non-linear transition function to predict the state
        self.x = self.f(self.x, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        # Use non-linear measurement function to update the state
        y = z - self.h(self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

# Define non-linear state transition and measurement functions (Example)
def f(x, u):
    # Non-linear state transition: Example (state includes position, velocity, acceleration)
    dt = 1.0 / 10  # 10 Hz update rate
    x_new = np.zeros_like(x)
    x_new[0] = x[0] + x[1] * dt + 0.5 * x[2] * dt**2  # Position update
    x_new[1] = x[1] + x[2] * dt  # Velocity update
    x_new[2] = x[2]  # Acceleration update (constant)
    return x_new

def h(x):
    # Non-linear measurement function: Example (measuring position only)
    return x[0]  # Only the position is measured

# Jacobian matrices for non-linear models
def F(x, u):
    # Jacobian of the state transition function (partial derivatives)
    dt = 1.0 / 10  # 10 Hz update rate
    return np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])

def H(x):
    # Jacobian of the measurement function (partial derivatives)
    return np.array([[1, 0, 0]])

# Kalman Filter Initialization
dt = 1.0 / 10  # 10 Hz update rate
Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])  # Process noise covariance
R = np.array([0.5]).reshape(1, 1)  # Measurement noise covariance

# Initialize Extended Kalman Filter for both x and y
kf_x = ExtendedKalmanFilter(f=f, h=h, F=F(np.zeros((3, 1)), 0), H=H(np.zeros((3, 1))), Q=Q, R=R)
kf_y = ExtendedKalmanFilter(f=f, h=h, F=F(np.zeros((3, 1)), 0), H=H(np.zeros((3, 1))), Q=Q, R=R)

try:
    while True:
        # Read and decode data
        data = DWM.readline().decode("utf-8").strip()

        if data and "POS" in data:
            data = data.replace("\r\n", "").split(",")
            try:
                raw_x = float(data[data.index("POS")+1])
                raw_y = float(data[data.index("POS")+2])

                # Predict the next state using the EKF
                kf_x.predict()
                kf_y.predict()

                # Update the filter with the new measurement
                kf_x.update(np.array([[raw_x]]))
                kf_y.update(np.array([[raw_y]]))

                # Get the filtered position estimates
                filtered_x = float(kf_x.x[0])
                filtered_y = float(kf_y.x[0])

                # Prepare position as JSON
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
