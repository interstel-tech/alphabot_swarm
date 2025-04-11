#!/usr/bin/env python
#-----------------------------------------------------------------------------
# ex1_qwiic_ICM20948.py (Tracks X-axis angle and estimated X/Y position)
#-----------------------------------------------------------------------------

import qwiic_icm20948
import time
import sys

def convert_imu_data(IMU):
    """
    Converts raw IMU data to physical units:
    - Accelerometer: m/s²
    - Gyroscope: °/s (only X-axis used)
    """
    ACCEL_SCALE_MODIFIER = 16384.0  # for ±2g range
    GYRO_SCALE_MODIFIER = 131.0     # for ±250 °/s
    GRAVITY = 9.80665               # m/s² per g

    # Convert accelerometer readings to m/s²
    ax = (IMU.axRaw / ACCEL_SCALE_MODIFIER) * GRAVITY
    ay = (IMU.ayRaw / ACCEL_SCALE_MODIFIER) * GRAVITY
    az = (IMU.azRaw / ACCEL_SCALE_MODIFIER) * GRAVITY

    # Convert gyroscope X reading to °/s
    gx = IMU.gxRaw / GYRO_SCALE_MODIFIER

    return {
        "accel_mps2": {"x": ax, "y": ay, "z": az},
        "gyro_dps_x": gx
    }

def runExample():
    print("\nSparkFun 9DoF ICM-20948 Sensor  Example (tracking X-angle + X/Y position)\n")
    IMU = qwiic_icm20948.QwiicIcm20948()

    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", file=sys.stderr)
        return

    IMU.begin()

    # Initialize angle, velocity, and position for X and Y axes
    angle_x = 0.0
    velocity = {"x": 0.0, "y": 0.0}
    position = {"x": 0.0, "y": 0.0}
    last_time = time.time()

    while True:
        if IMU.dataReady():
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            IMU.getAgmt()  # update sensor values
            converted = convert_imu_data(IMU)

            # Update angle
            angle_x += converted["gyro_dps_x"] * dt

            # Basic dead reckoning (double integration of acceleration)
            # Assumes acceleration has been gravity-compensated (it hasn't!)
            accel_x = converted["accel_mps2"]["x"]
            accel_y = converted["accel_mps2"]["y"]

            # Integrate acceleration to velocity
            velocity["x"] += accel_x * dt
            velocity["y"] += accel_y * dt

            # Integrate velocity to position
            position["x"] += velocity["x"] * dt
            position["y"] += velocity["y"] * dt

            print("Accel (m/s²): X: {:.2f}, Y: {:.2f}, Z: {:.2f} | Gyro X (°/s): {:.2f}".format(
                accel_x, accel_y, converted["accel_mps2"]["z"], converted["gyro_dps_x"]
            ))

            print("Estimated X-Axis Angle (°): {:.2f}".format(angle_x))
            print("Estimated Position (m): X: {:.4f}, Y: {:.4f}\n".format(position["x"], position["y"]))

            time.sleep(0.1)
        else:
            print("Waiting for data")
            time.sleep(0.5)

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit):
        print("\nEnding Example")
        sys.exit(0)
