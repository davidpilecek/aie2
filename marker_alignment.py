import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
import serial
import time
import driving_functions as dfu
from scipy.spatial.transform import Rotation as R

import math
import matplotlib
import matplotlib.pyplot as plt

from libcamera import controls
from simple_pid import PID

matplotlib.use("Agg")

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 360))

# Define Kalman Filter
class PoseKalmanFilter:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(6, 6)  # 6 state vars, 6 measurement vars

        # State: [x, y, z, roll, pitch, yaw]
        # Measurement: [x, y, z, roll, pitch, yaw]

        # Transition matrix (assuming constant velocity model)
        self.kalman.transitionMatrix = np.eye(6, dtype=np.float32)

        # Measurement matrix
        self.kalman.measurementMatrix = np.eye(6, dtype=np.float32)

        # Process noise covariance
        self.kalman.processNoiseCov = np.eye(6, dtype=np.float32) * KALMAN_PROCESS_COEF

        # Measurement noise covariance
        self.kalman.measurementNoiseCov = np.eye(6, dtype=np.float32) * KALMAN_MEASUREMENT_COEF

        # Error covariance
        self.kalman.errorCovPost = np.eye(6, dtype=np.float32) * KALMAN_ERROR_COEF

        # Initial state (e.g., zeros)
        self.kalman.statePost = np.zeros(6, dtype=np.float32)

    def predict(self):
        return self.kalman.predict()

    def correct(self, measurement):
        return self.kalman.correct(measurement)

# Create an instance of the Kalman filter
pose_filter = PoseKalmanFilter()

def smooth_pose_estimation(corners, ids, rvecs, tvecs):
    smoothed_poses = []

    for i in range(len(ids)):
        # Extract raw measurements (translation and rotation vectors)
        tvec = tvecs[i].flatten()
        rvec = rvecs[i].flatten()
        
        # Convert rotation vector to Euler angles (roll, pitch, yaw)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        euler_angles = cv2.decomposeProjectionMatrix(np.hstack((rotation_matrix, [[0], [0], [0]])))[-1]
        roll, yaw, pitch = euler_angles.flatten()
        
        real_yaw = yaw
        
        # Construct measurement vector: [x, y, z, roll, pitch, yaw]
        measurement = np.array([tvec[0], tvec[1], tvec[2], roll, pitch, yaw], dtype=np.float32)

        # Predict the next state
        predicted = pose_filter.predict()

        # Update the Kalman filter with the current measurement
        corrected = pose_filter.correct(measurement)

        # Save the corrected pose for use
        smoothed_poses.append(corrected[:6])

    return smoothed_poses, real_yaw


#PID settings for each parameter of marker alignment
pid_centre = PID(0.1, 0.02, 0, setpoint = centre_of_frame[0])
pid_centre.sample_time = 0.01
pid_centre.output_limits = (-3, 0)

pid_angle = PID(0.1,0, 0, setpoint = 180)
pid_angle.sample_time = 0.01
pid_angle.output_limits = (-2, 2)

pid_distance = PID(0.1, 0, 0, setpoint = DISTANCE_FROM_MARKER)
pid_distance.sample_time = 0.01
pid_distance.output_limits = (-3, 0)

#camera config
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main = {"size": (640, 360)}))
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

aligned_translation = False
aligned_rotation = False
aligned_distance = False

marker_state_holder = False            #holder for if marker is lost
seeing_marker = False
marker_lost = True

last_marker_position = 0             # -1 = left, 1 = right 



def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # in radians
 
def get_marker_centre(marker_id):
    centre_x1 = (int(corners[marker_id][0][0][0])+int(corners[marker_id][0][1][0]))/2
    centre_x2 = (int(corners[marker_id][0][2][0])+int(corners[marker_id][0][3][0]))/2
    centre_x = int((centre_x1+centre_x2)/2)

    centre_y1 = (int(corners[marker_id][0][0][1])+int(corners[marker_id][0][1][1]))/2
    centre_y2 = (int(corners[marker_id][0][2][1])+int(corners[marker_id][0][3][1]))/2
    centre_y = int((centre_y1+centre_y2)/2) 
    
    return centre_x, centre_y
    
    
#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
aruco_marker_side_length = ARUCO_MARKER_SIZE / (1280/FRAME_DIMENSIONS[0])

# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'

# Load the camera parameters from the saved file
cv_file = cv2.FileStorage(
    camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()
    
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()
time.sleep(2)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 360))


while True:
    
    print("\n")
    print(f"marker_lost state: {marker_lost}")
    if aligned_translation:
        color_of_center = (0,255,0)
        
    else:
        color_of_center = (0,0,255)
    
    # Capture frame
    frame = picam2.capture_array()
    # Operations on the frame
    frame = cv2.resize(frame, FRAME_DIMENSIONS)
    frame = cv2.flip(frame, -1)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    #cv2.rectangle(frame, (centre_of_frame[0] - MARGIN_OF_CENTER , centre_of_frame[1] - MARGIN_OF_CENTER), (centre_of_frame[0] + MARGIN_OF_CENTER , centre_of_frame[1] + MARGIN_OF_CENTER), color_of_center, 3)
    cv2.line(frame, (centre_of_frame[0] - MARGIN_OF_CENTER_MISALIGNMENT , 0), (centre_of_frame[0] - MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_center, 3)
    cv2.line(frame, (centre_of_frame[0] + MARGIN_OF_CENTER_MISALIGNMENT , 0), (centre_of_frame[0] + MARGIN_OF_CENTER_MISALIGNMENT , FRAME_DIMENSIONS[1]), color_of_center, 3)
    
    corners, marker_ids, rejectedImgPoints = detector.detectMarkers(gray)

    if marker_ids is not None:
        if marker_state_holder == False:
            marker_state_holder = True
        marker_lost = False
        
        
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids, borderColor=(255, 0, 0))
           # Get the rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        mtx,
        dst)
        
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards the ground
      # z-axis points straight ahead away from your eye, out of the camera
        for i, marker_id in enumerate(marker_ids):
       
        # Store the translation (i.e. position) information
            transform_translation_x = tvecs[i][0][0] 
            transform_translation_y = tvecs[i][0][1]
        #multiply by 100 to convert to cm
            transform_translation_z = tvecs[i][0][2] * 100 
     
            # Store the rotation information
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat()   
             
            # Quaternion format     
            transform_rotation_x = quat[0] 
            transform_rotation_y = quat[1] 
            transform_rotation_z = quat[2] 
            transform_rotation_w = quat[3] 
             
            # Euler angle format in radians
            roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                           transform_rotation_y, 
                                                           transform_rotation_z, 
                                                           transform_rotation_w)
             
            roll_x_deg = round(math.degrees(roll_x), 3)
            print(roll_x_deg)
            pitch_y = math.degrees(pitch_y)
            yaw_z = math.degrees(yaw_z)
            # ~ print("transform_translation_x: {}".format(transform_translation_x))
            print("transform_translation_z: {}".format(transform_translation_z))
            # ~ print(f"roll_x in deg: {roll_x_deg}")
            #print("pitch_y: {}".format(pitch_y))
            #print("yaw_z: {}".format(yaw_z))
             
            # Draw the axes on the marker
            cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
        try:            
            centre_x, centre_y = get_marker_centre(0)
            cv2.putText(frame, ".", (centre_x, centre_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            SPEED_STRAFE_PID = int(SPEED_STRAFE + pid_centre(centre_x))
            SPEED_ROLL_PID = int(SPEED_ROLL + pid_angle(roll_x_deg))
            print(SPEED_STRAFE_PID)
            
            # ~ if centre_x < centre_of_frame[0] - MARGIN_OF_CENTER_MISALIGNMENT:
                # ~ aligned_translation = False
                # ~ last_marker_position = -1
                # ~ dfu.strafe_left(SPEED_STRAFE_PID, ser)
                # ~ cv2.putText(frame, f"strafe_l", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                
            # ~ elif centre_x > centre_of_frame[0] + MARGIN_OF_CENTER_MISALIGNMENT:
                # ~ aligned_translation = False
                # ~ last_marker_position = 1
                # ~ dfu.strafe_right(SPEED_STRAFE_PID, ser)
                # ~ cv2.putText(frame, f"strafe_r", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                
            # ~ else:
                # ~ last_marker_position = 0
                # ~ aligned_translation = True
                # ~ print("translation aligned")
                # ~ dfu.stop_all(ser)
                
            if 90 < roll_x_deg < 180 - MARGIN_OF_ANGLE:
                    aligned_rotation = False
                    dfu.roll_left(SPEED_ROLL_PID, ser)
                    cv2.putText(frame, f"roll_l", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                    
            elif -180 + MARGIN_OF_ANGLE < roll_x_deg < -90:
                    aligned_rotation = False
                    dfu.roll_right(SPEED_ROLL_PID, ser)
                    cv2.putText(frame, f"roll_l", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                    
            else:
                    aligned_rotation = True
                    print("rotation aligned")
                    dfu.stop_all(ser)

            # ~ if aligned_translation and aligned_rotation:

                # ~ if transform_translation_z > DISTANCE_FROM_MARKER + MARGIN_OF_DISTANCE:
                    # ~ aligned_distance = False
                    # ~ dfu.drive_forward(SPEED_DRIVE, ser)
                    # ~ cv2.putText(frame, f"drive_F", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                    
                # ~ elif transform_translation_z < DISTANCE_FROM_MARKER - MARGIN_OF_DISTANCE:
                    # ~ aligned_distance = False
                    # ~ dfu.drive_reverse(SPEED_DRIVE, ser)
                    # ~ cv2.putText(frame, f"drive_R", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)
                # ~ else:
                    # ~ aligned_distance = True
                    # ~ dfu.stop_all(ser)
                    # ~ print("all aligned")
    
        except Exception as e:
            print(e)
            pass
    else:
        dfu.stop_all(ser)
        marker_lost = True 
        
    if marker_lost:
        if last_marker_position == 1:
            dfu.strafe_right(SPEED_STRAFE, ser)
            print("going right")
        elif last_marker_position == -1:
            dfu.strafe_left(SPEED_STRAFE, ser)
            print("going left")
        elif last_marker_position == 0:
            dfu.spin_left(SPEED_SPIN, ser)
    # Display the resulting frame
    cv2.imshow('frame', frame)
    out.write(frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
dfu.stop_all(ser)
picam2.stop()
cv2.destroyAllWindows()
