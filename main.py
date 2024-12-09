import cv2

import serial
import time
import driving_functions as dfu
from scipy.spatial.transform import Rotation as R
import scipy.signal

import math
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

from simple_pid import PID

from kalman import *
from config import *
from camera_functions import *

from collections import deque

matplotlib.use("Agg")

pre_filter = deque(maxlen=5)
post_filter = deque(maxlen=5)

#state variables
aligned_center = False
aligned_rotation = False
aligned_distance = False

marker_state_holder = False            
seeing_marker = False
marker_lost = True

last_marker_position = 0             # -1 = left, 1 = right, 0 = no marker seen

select_task = 1                      # no tasks = 0; line following = 1; marker alignment = 2


#PID settings for each parameter of marker alignment
pid_centre = PID(0.1, 0.02, 0, setpoint = CENTRE_OF_FRAME[0])
pid_centre.sample_time = 0.01
pid_centre.output_limits = (-1, 0)

pid_angle = PID(0.1, 0.02, 0.001, setpoint = 0)
pid_angle.sample_time = 0.01
pid_angle.output_limits = (-2, 2)

pid_distance = PID(0.1, 0, 0, setpoint = DISTANCE_FROM_MARKER)
pid_distance.sample_time = 0.01
pid_distance.output_limits = (-3, 0)

#initialize camera
picam2 = start_camera()

#initialize aruco detector
detector, mtx, dst = start_aruco()

#init video capture
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 360))

#initialize kalman filter
pose_filter = PoseKalmanFilter()

#initialize serial communication
arduino_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
arduino_port.reset_input_buffer()
time.sleep(2)

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
    print(f"task selection: {select_task}")
    
    print(f"aligned_center: {aligned_center}")
    print(f"aligned_rotation: {aligned_rotation}")
    print(f"aligned_distance: {aligned_distance}")
    print(f"marker_lost {marker_lost}")

    #capture frame and convert to grayscale
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_blurred = cv2.GaussianBlur(frame_gray, (21, 21), 0)
    
    serial_port = arduino_port
    
    corners, marker_ids, rejectedImgPoints = detector.detectMarkers(frame_gray)
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids, borderColor=(255, 0, 0))
                        
            #detect markers               
    if marker_ids is not None:
        marker_lost = False
            # Get the rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        ARUCO_MARKER_SIZE,
        mtx,
        dst)
        
        smoothed_poses, real_yaw, yaw_mean = smooth_pose_estimation(marker_ids, rvecs, tvecs, pose_filter, pre_filter, post_filter)

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

                #distances from tag in cm
                transform_translation_x = tvec[0] * 100
                transform_translation_y = tvec[1] * 100
                translation_z = tvec[2] * 100

                cv2.putText(frame, f"translation z: {translation_z:.2f}", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, f"yaw deg: {yaw_deg:.2f}", (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, f"real yaw deg: {real_yaw:.2f}", (0, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                
                if translation_z < 50:
                    select_task = 2
                

                cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05) 
        else:
            marker_lost = True  
        
    if(select_task == 2):
        if marker_ids is not None:
            marker_lost = False
            if marker_state_holder == False:
                marker_state_holder = True
            
            if aligned_center:
                color_of_line = (0,255,0)
            else:
                color_of_line = (0,0,255)
            try:
                centre_x, centre_y = get_marker_centre(0, corners)
                        
                #show borders for translation alignment of marker
                cv2.line(frame, (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT , 0), (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_line, 3)
                cv2.line(frame, (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT , 0), (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_line, 3)
                speed_strafe_pid = int(SPEED_STRAFE + pid_centre(centre_x))
                speed_drift_pid = int(SPEED_DRIFT + pid_angle(yaw_deg))
                speed_drive_pid = int(SPEED_DRIVE + pid_distance(translation_z))
                print(f"pid stafe: {speed_strafe_pid}")
                speed_strafe = SPEED_STRAFE
                speed_drift = SPEED_DRIFT
                speed_drive = SPEED_DRIVE
                
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

                    
                if MARGIN_OF_ANGLE < yaw_combined < 90:
                    aligned_rotation = False
                    dfu.drift_left(speed_drift_pid, serial_port)
                elif -90 < yaw_combined < -MARGIN_OF_ANGLE:
                    aligned_rotation = False
                    dfu.drift_right(speed_drift_pid, serial_port)
                else:
                    aligned_rotation = True
                    
                    
                if translation_z > DISTANCE_FROM_MARKER + MARGIN_OF_DISTANCE:
                    aligned_distance = False
                    dfu.drive_forward(speed_drive, serial_port)
                elif translation_z < DISTANCE_FROM_MARKER - MARGIN_OF_DISTANCE:
                    aligned_distance = False
                    dfu.drive_reverse(speed_drive, serial_port)
                else:
                    aligned_distance = True
                    
                if aligned_center and aligned_distance and aligned_rotation:
                    dfu.stop_all(arduino_port)
                    #select_task = 0
                    print("all aligned")
            except Exception as e:
                print(e)
                dfu.stop_all(arduino_port)
        else:
            marker_lost = True   
            #dfu.stop_all(arduino_port)
        if marker_lost:
            if last_marker_position == 1:
                dfu.strafe_right(SPEED_STRAFE, arduino_port)
                print("looking right")
            elif last_marker_position == -1:
                dfu.strafe_left(SPEED_STRAFE, arduino_port)
                print("looking left")
            elif last_marker_position == 0:
                dfu.spin_right(SPEED_SPIN, arduino_port)

    elif(select_task == 1):
        masked_image, frame_draw = create_mask(frame, frame_blurred, frame_gray)
        try:
            contours, max_contour, frame_draw = get_contours(masked_image, frame_draw)

            if len(contours)>0:
                line_angle, frame_draw = get_line_angle(max_contour, frame_draw)  
                offset, frame_draw = get_offset(max_contour, frame_draw)
                dfu.turn(line_angle, offset, arduino_port)
            else:
                print("no contours")
                dfu.stop_all(arduino_port)
        except Exception as e:
            print(e)

        frame = frame_draw

    #Display the final frame
    cv2.imshow('frame', frame)
    #add frame to video
    out.write(frame)
    #if q key pressed, break loop and stop process
    if cv2.waitKey(1) == ord('q'):
        break

dfu.stop_all(arduino_port)
picam2.stop()
cv2.destroyAllWindows()
    
    
figure, axis = plt.subplots(2, 2, figsize=(8, 6))
#plt.plot(high_boundary)
#plt.plot(low_boundary)

df1 = pd.DataFrame(yaw_real, columns = ["real yaw"])
df2 = pd.DataFrame(yaw_mean_list, columns = ["mean yaw"])
df3 = pd.DataFrame(yaw_corrected, columns = ["kalman yaw"])
df4 = pd.DataFrame(yaw_combined_list, columns = ["combined yaw"])

df1.to_excel("output1.xlsx", index=False)
df2.to_excel("output2.xlsx", index=False)
df3.to_excel("output3.xlsx", index=False)
df4.to_excel("output4.xlsx", index=False)


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

