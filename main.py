import cv2
from picamera2 import Picamera2
from libcamera import controls

import serial
import time
import driving_functions as dfu
from scipy.spatial.transform import Rotation as R

import math
import numpy as np

import matplotlib
import matplotlib.pyplot as plt

from simple_pid import PID

from kalman import *
from config import *

#state variables
aligned_translation = False
aligned_rotation = False
aligned_distance = False

marker_state_holder = False            
seeing_marker = False
marker_lost = True

last_marker_position = 0             # -1 = left, 1 = right, 0 = no marker seen

# line following = 0; marker alignment = 1
select_task = 1

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


while True:
    #capture frame and convert to grayscale
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if aligned_translation:
        color_of_line = (0,255,0)
    else:
        color_of_line = (0,0,255)

    #detect markers
    corners, marker_ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    #show borders for translation alignment of marker
    if(select_task == 0):
        cv2.line(frame, (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT , 0), (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_line, 3)
        cv2.line(frame, (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT , 0), (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_line, 3)
    
        if marker_ids is not None:
            if marker_state_holder == False:
                marker_state_holder = True
            marker_lost = False

            frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids, borderColor=(255, 0, 0))
               # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            ARUCO_MARKER_SIZE,
            mtx,
            dst)
            centre = get_marker_centre(0, corners)
            smoothed_poses, real_yaw = smooth_pose_estimation(marker_ids, rvecs, tvecs, pose_filter)

            for i, marker_id in enumerate(marker_ids):

                    smoothed_pose = smoothed_poses[i]
                    tvec = smoothed_pose[:3]
                    roll_deg, pitch_deg, yaw_deg = smoothed_pose[3:]
                    
                    roll_deg = round(roll_deg, 2)
                    pitch_deg = round(pitch_deg, 2)
                    yaw_deg = round(yaw_deg, 2)

                    # ~ yaw_corrected.append(yaw_deg)
                    # ~ yaw_real.append(real_yaw)
                    
                    # ~ trans_corrected.append(centre_corrected)
                    # ~ trans_real.append(centre[0])
                    
                    # ~ print(f"roll deg: {roll_deg:.2f}")
                    # ~ print(f"pitch deg: {pitch_deg:.2f}")
                    # ~ print(f"yaw deg: {yaw_deg:.2f}")
                    
                    # ~ cv2.putText(frame, f"roll deg: {roll_deg:.2f}", (0, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                    # ~ cv2.putText(frame, f"pitch deg: {pitch_deg:.2f}", (0, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(frame, f"yaw deg: {yaw_deg:.2f}", (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                    
                    #distances from tag in cm
                    transform_translation_x = tvec[0] * 100
                    transform_translation_y = tvec[1] * 100
                    transform_translation_z = tvec[2] * 100
                    
                    cv2.putText(frame, f"translation z: {transform_translation_z:.2f}", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

                    cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)


    #Display the final frame
    cv2.imshow('frame', frame)
    #add frame to video
    out.write(frame)
    #if q key pressed, break loop and stop process
    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
    
