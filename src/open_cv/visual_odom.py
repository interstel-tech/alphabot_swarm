import cv2
import numpy as np

# Camera calibration parameters (based on camera_calibration.py)
camera_matrix = np.array([[3.94465561e+03, 0.00000000e+00, 8.97331509e+02],
                          [0.00000000e+00, 4.01204355e+03, 1.20645873e+03],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)

dist_coeffs = np.array([[-3.07803245e-01, -2.65799922e+00, -7.94542198e-03, 8.74052459e-03, 1.75946526e+01]], dtype=np.float32)

# Set up the video capture
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Use V4L2 instead of GStreamer

# Variables for visual odometry
prev_frame = None
prev_points = None
camera_pos = np.zeros((3, 1))  # Initialize camera position (x, y, z)
total_translation = np.zeros((3, 1))  # Accumulated translation in 3D space

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Undistort the frame using the camera matrix and distortion coefficients
    frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # If it's the first frame, set it up
    if prev_frame is None:
        prev_frame = gray
        prev_points = cv2.goodFeaturesToTrack(prev_frame, mask=None, **{
            'maxCorners': 500,
            'qualityLevel': 0.3,
            'minDistance': 7,
            'blockSize': 7
        })

        # Check if no points are detected
        if prev_points is None or len(prev_points) == 0:
            print("No good features detected, skipping frame")
            continue

        prev_points = prev_points.reshape(-1, 1, 2)  # Reshape the points for optical flow
        continue

    # Calculate optical flow (i.e., find new feature positions in the next frame)
    next_points, status, err = cv2.calcOpticalFlowPyrLK(prev_frame, gray, prev_points, None)

    # Check if there are valid points to process
    if next_points is None or status is None or len(next_points) == 0:
        print("No points to track, skipping frame")
        prev_frame = gray
        continue

    # Select good points
    good_old = prev_points[status == 1]
    good_new = next_points[status == 1]

    # Estimate motion using the essential matrix
    E, mask = cv2.findEssentialMat(good_new, good_old, camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, mask = cv2.recoverPose(E, good_new, good_old, camera_matrix)

    # Update the camera position
    camera_pos += t  # Update camera position based on translation
    total_translation += t  # Accumulate translation for total movement

    # Draw the tracking points and motion (for visualization)
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        frame = cv2.line(frame, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
        frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 0, 255), -1)

    # Display the frame with tracking points
    cv2.imshow('Frame', frame)

    # Update previous frame and points for next iteration
    prev_frame = gray
    prev_points = good_new.reshape(-1, 1, 2)

    # Show the current estimated camera position (x, y, z)
    print(f"Estimated camera position: X={camera_pos[0][0]}, Y={camera_pos[1][0]}, Z={camera_pos[2][0]}")

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
