import cv2
import serial
import time
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import scipy.signal
from scipy.spatial.transform import Rotation as R
from collections import deque
from simple_pid import PID

# Import custom modules
import driving_functions as dfu
from kalman import *
from config import *
from camera_functions import *

# Configure matplotlib
matplotlib.use("Agg")

# Initialize filter queues
pre_filter = deque(maxlen=5)
post_filter = deque(maxlen=5)

# State variables
aligned_center = False
aligned_rotation = False
aligned_distance = False
marker_state_holder = False
seeing_marker = False
marker_lost = True
last_marker_position = 0  # -1 = left, 1 = right, 0 = no marker seen
select_task = 1  # 0 = no tasks; 1 = line following; 2 = marker alignment

# PID settings for marker alignment
pid_centre = PID(0.1, 0.02, 0, setpoint=CENTRE_OF_FRAME[0])
pid_centre.sample_time = 0.01
pid_centre.output_limits = (-1, 0)

pid_angle = PID(0.1, 0.02, 0.001, setpoint=0)
pid_angle.sample_time = 0.01
pid_angle.output_limits = (-2, 2)

pid_distance = PID(0.1, 0, 0, setpoint=DISTANCE_FROM_MARKER)
pid_distance.sample_time = 0.01
pid_distance.output_limits = (-3, 0)

# Initialize camera
picam2 = start_camera()

# Initialize ArUco detector
detector, mtx, dst = start_aruco()

# Initialize video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 360))

# Initialize Kalman filter
pose_filter = PoseKalmanFilter()

# Initialize serial communication
arduino_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
arduino_port.reset_input_buffer()
time.sleep(2)

# Yaw and pose variables
current_yaw = 0
previous_yaw = 0
yaw_mean = 0
yaw_mean_list = []
yaw_combined_list = []
yaw_corrected = []
yaw_real = []
translation_z = 15

while True:
    print("\n")
    print(f"Task selection: {select_task}")
    print(f"Aligned center: {aligned_center}")
    print(f"Aligned rotation: {aligned_rotation}")
    print(f"Aligned distance: {aligned_distance}")
    print(f"Marker lost: {marker_lost}")

    # Capture frame and preprocess
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_blurred = cv2.GaussianBlur(frame_gray, (21, 21), 0)
    serial_port = arduino_port

    # Detect markers
    corners, marker_ids, rejected_img_points = detector.detectMarkers(frame_gray)
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids, borderColor=(255, 0, 0))

    if marker_ids is not None:
        marker_lost = False
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners, ARUCO_MARKER_SIZE, mtx, dst
        )

        smoothed_poses, real_yaw, yaw_mean = smooth_pose_estimation(
            marker_ids, rvecs, tvecs, pose_filter, pre_filter, post_filter
        )

        for i, marker_id in enumerate(marker_ids):
            smoothed_pose = smoothed_poses[i]
            tvec = smoothed_pose[:3]
            drift_deg, yaw_combined, yaw_deg = smoothed_pose[3:]

            drift_deg = round(drift_deg, 2)
            yaw_combined = round(yaw_combined, 2)
            yaw_deg = round(yaw_deg, 2)

            yaw_corrected.append(yaw_deg)
            yaw_real.append(real_yaw)
            yaw_mean_list.append(yaw_mean)
            yaw_combined_list.append(yaw_combined)

            # Distances from tag in cm
            transform_translation_x = tvec[0] * 100
            transform_translation_y = tvec[1] * 100
            translation_z = tvec[2] * 100

            cv2.putText(frame, f"Translation Z: {translation_z:.2f}", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Yaw deg: {yaw_deg:.2f}", (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

            if translation_z < 50:
                select_task = 2

            cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
    else:
        marker_lost = True

    # Task: Marker alignment
    if select_task == 2:
        if marker_ids is not None:
            marker_lost = False
            if not marker_state_holder:
                marker_state_holder = True

            color_of_line = (0, 255, 0) if aligned_center else (0, 0, 255)

            try:
                centre_x, centre_y = get_marker_centre(0, corners)

                # Show borders for translation alignment of marker
                cv2.line(frame, (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT, 0),
                         (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT, FRAME_DIMENSIONS[1]), color_of_line, 3)
                cv2.line(frame, (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT, 0),
                         (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT, FRAME_DIMENSIONS[1]), color_of_line, 3)

                # Calculate PID values for each movement
                speed_strafe_pid = int(SPEED_STRAFE + pid_centre(centre_x))
                speed_drift_pid = int(SPEED_DRIFT + pid_angle(yaw_combined))
                speed_drive_pid = int(SPEED_DRIVE + pid_distance(translation_z))

                # Alignment of center of robot with marker
                if centre_x < CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT:
                    aligned_center = False
                    last_marker_position = -1
                    dfu.strafe_left(speed_strafe_pid, serial_port)
                elif centre_x > CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT:
                    aligned_center = False
                    last_marker_position = 1
                    dfu.strafe_right(speed_strafe_pid, serial_port)
                else:
                    aligned_center = True

                # Alignment of rotation of robot with marker
                if MARGIN_OF_ANGLE < yaw_combined < 90:
                    aligned_rotation = False
                    dfu.drift_left(speed_drift_pid, serial_port)
                elif -90 < yaw_combined < -MARGIN_OF_ANGLE:
                    aligned_rotation = False
                    dfu.drift_right(speed_drift_pid, serial_port)
                else:
                    aligned_rotation = True

                # Alignment of distance of robot with marker
                if translation_z > DISTANCE_FROM_MARKER + MARGIN_OF_DISTANCE:
                    aligned_distance = False
                    dfu.drive_forward(speed_drive_pid, serial_port)
                elif translation_z < DISTANCE_FROM_MARKER - MARGIN_OF_DISTANCE:
                    aligned_distance = False
                    dfu.drive_reverse(speed_drive_pid, serial_port)
                else:
                    aligned_distance = True

                # Stop all movement if fully aligned
                if aligned_center and aligned_rotation and aligned_distance:
                    dfu.stop_all(serial_port)
                    print("All aligned")

            except Exception as e:
                print(e)
                dfu.stop_all(serial_port)

        # If no marker is detected
        elif marker_lost:
            if last_marker_position == 1:
                dfu.strafe_right(SPEED_STRAFE, serial_port)
                print("Looking right")
            elif last_marker_position == -1:
                dfu.strafe_left(SPEED_STRAFE, serial_port)
                print("Looking left")
            elif last_marker_position == 0:
                print("Looking for marker")
                dfu.spin_right(SPEED_SPIN, serial_port)

    # Task: Line following
    elif select_task == 1:
        masked_image, frame_draw = create_mask(frame, frame_blurred, frame_gray)

        try:
            contours, max_contour, frame_draw = get_contours(masked_image, frame_draw)
            if contours:
                line_angle, frame_draw = get_line_angle(max_contour, frame_draw)
                offset, frame_draw = get_offset(max_contour, frame_draw)
                dfu.turn(line_angle, offset, serial_port)
            else:
                print("No contours detected")
        except Exception as e:
            print(e)

    # Display the frame
    cv2.imshow('Frame', frame)
    out.write(frame)

    # Exit on 'q'
    if cv2.waitKey(1) == ord('q'):
        break

# Cleanup
dfu.stop_all(arduino_port)
picam2.stop()
cv2.destroyAllWindows()

#saving data about effect of filtering techniques on marker yaw and exporting as graphs for debugging purposes
df1 = pd.DataFrame(yaw_real, columns = ["real yaw"])
df2 = pd.DataFrame(yaw_mean_list, columns = ["mean yaw"])
df3 = pd.DataFrame(yaw_corrected, columns = ["kalman yaw"])
df4 = pd.DataFrame(yaw_combined_list, columns = ["combined yaw"])

df1.to_excel("output1.xlsx", index=False)
df2.to_excel("output2.xlsx", index=False)
df3.to_excel("output3.xlsx", index=False)
df4.to_excel("output4.xlsx", index=False)

figure, axis = plt.subplots(2, 2, figsize=(8, 6))

axis[0, 0].plot(yaw_real, label="real yaw", color = "red")
axis[0, 0].plot(yaw_mean_list, label="median yaw", color = "blue")
axis[0, 0].set_title("Median Filtering")
axis[0, 1].plot(yaw_real, label="real yaw", color = "red")
axis[0, 1].plot(yaw_corrected, label="Kalman-filtered yaw", color = "green")
axis[0, 1].set_title("Kalman Filtering")
axis[1, 0].plot(yaw_real, label="real yaw", color = "red")
axis[1, 0].plot(yaw_combined_list, label="filtered yaw", color = "purple")
axis[1, 0].plot(yaw_mean_list, label="median yaw", color = "blue")
axis[1, 0].set_title("Median + Kalman Filtering")

axis[0, 0].legend()
axis[0, 1].legend()
axis[0, 0].grid()
axis[0, 1].grid()
axis[1, 0].legend()
axis[1, 0].grid()
plt.savefig("aruco__kalman_graph0.png")