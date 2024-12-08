import cv2

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
from camera_functions import *

#state variables
aligned_center = False
aligned_rotation = False
aligned_distance = False

marker_state_holder = False            
seeing_marker = False
marker_lost = True


last_marker_position = 0             # -1 = left, 1 = right, 0 = no marker seen

select_task = 0                      # marker alignment = 0; line following = 1


#PID settings for each parameter of marker alignment
pid_centre = PID(0.1, 0.02, 0, setpoint = CENTRE_OF_FRAME[0])
pid_centre.sample_time = 0.01
pid_centre.output_limits = (-3, 0)

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


while True:
    #capture frame and convert to grayscale
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_blurred = cv2.GaussianBlur(frame_gray, (21, 21), 0)


    corners, marker_ids, rejectedImgPoints = detector.detectMarkers(frame_gray)
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids, borderColor=(255, 0, 0))
            
            
    if marker_ids is not None:
        if marker_state_holder == False:
            marker_state_holder = True
        marker_lost = False
            #detect markers               
        
            # Get the rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        ARUCO_MARKER_SIZE,
        mtx,
        dst)
        centre_x, centre_y = get_marker_centre(0, corners)
        cv2.putText(frame, ".", (centre_x, centre_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 5, cv2.LINE_AA)
                
        smoothed_poses, real_yaw = smooth_pose_estimation(marker_ids, rvecs, tvecs, pose_filter)

        for i, marker_id in enumerate(marker_ids):

                smoothed_pose = smoothed_poses[i]
                tvec = smoothed_pose[:3]
                drift_deg, pitch_deg, yaw_deg = smoothed_pose[3:]
                
                drift_deg = round(drift_deg, 2)
                pitch_deg = round(pitch_deg, 2)
                yaw_deg = round(yaw_deg, 2)

                # ~ yaw_corrected.append(yaw_deg)
                # ~ yaw_real.append(real_yaw)
                
                # ~ trans_corrected.append(centre_corrected)
                # ~ trans_real.append(centre[0])
                
                # ~ print(f"drift deg: {drift_deg:.2f}")
                # ~ print(f"pitch deg: {pitch_deg:.2f}")
                # ~ print(f"yaw deg: {yaw_deg:.2f}")
                
                # ~ cv2.putText(frame, f"drift deg: {drift_deg:.2f}", (0, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                # ~ cv2.putText(frame, f"pitch deg: {pitch_deg:.2f}", (0, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, f"yaw deg: {yaw_deg:.2f}", (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
                
                #distances from tag in cm
                transform_translation_x = tvec[0] * 100
                transform_translation_y = tvec[1] * 100
                translation_z = tvec[2] * 100



                cv2.putText(frame, f"translation z: {translation_z:.2f}", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

                cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)        

        speed_strafe_pid = int(SPEED_STRAFE + pid_centre(centre_x))
        speed_drift_pid = int(SPEED_DRIFT + pid_angle(yaw_deg))
        speed_drive_pid = int(SPEED_DRIVE + pid_distance(translation_z))
        
        speed_strafe = SPEED_STRAFE
        speed_drift = SPEED_DRIFT
        speed_drive = SPEED_DRIVE
        serial_port = arduino_port
    if(select_task == 0):
        
        if aligned_center:
            color_of_line = (0,255,0)
        else:
            color_of_line = (0,0,255)
        #show borders for translation alignment of marker
        cv2.line(frame, (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT , 0), (CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_line, 3)
        cv2.line(frame, (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT , 0), (CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_line, 3)
        try:
            
            if centre_x < CENTRE_OF_FRAME[0] - MARGIN_OF_CENTER_MISALIGNMENT:
                aligned_center = False
                last_marker_position = -1
                dfu.strafe_left(speed_strafe, serial_port)
            elif centre_x > CENTRE_OF_FRAME[0] + MARGIN_OF_CENTER_MISALIGNMENT:
                aligned_center = False
                last_marker_position = 1
                dfu.strafe_right(speed_strafe, serial_port)
            else:
                last_marker_position = 0
                aligned_center = True
                print("translation aligned")
                
            if MARGIN_OF_ANGLE < yaw_deg < 90:
                aligned_rotation = False
                dfu.drift_left(speed_drift, serial_port)
            elif -90 < yaw_deg < -MARGIN_OF_ANGLE:
                aligned_rotation = False
                dfu.drift_right(speed_drift, serial_port)
            else:
                aligned_rotation = True
                print("rotation aligned")
                
            if translation_z > DISTANCE_FROM_MARKER + MARGIN_OF_DISTANCE:
                aligned_distance = False
                dfu.drive_forward(speed_drive, serial_port)
            elif translation_z < DISTANCE_FROM_MARKER - MARGIN_OF_DISTANCE:
                aligned_distance = False
                dfu.drive_reverse(speed_drive, serial_port)
            else:
                aligned_distance = True
                print("distance aligned")
                
            if aligned_center and aligned_rotation and aligned_distance:
                dfu.stop_all(arduino_port)
                print("all aligned")
                
        except Exception as e:
            print(e)

    elif(select_task == 1):

        masked_image, frame_draw = create_mask(frame, frame_blurred, frame_gray)
        try:
            contours, max_contour, frame_draw = get_contours(masked_image, frame_draw)
        except Exception as e:
            print(e)

        if len(contours)>0:
            line_angle, frame_draw = get_line_angle(max_contour, frame_draw)  
            offset, frame_draw = get_offset(max_contour, frame_draw)
            dfu.turn(line_angle, offset, arduino_port)

        else:
            print("no contours")
            dfu.stop_all(arduino_port)
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
    
