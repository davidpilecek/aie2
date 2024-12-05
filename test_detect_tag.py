import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
from scipy.spatial.transform import Rotation as R
import math # Math library
from libcamera import controls


import cv2
import numpy as np

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
        self.kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 3e-2

        # Measurement noise covariance
        self.kalman.measurementNoiseCov = np.eye(6, dtype=np.float32) * 1e-2

        # Error covariance
        self.kalman.errorCovPost = np.eye(6, dtype=np.float32)

        # Initial state (e.g., zeros)
        self.kalman.statePost = np.zeros(6, dtype=np.float32)

    def predict(self):
        return self.kalman.predict()

    def correct(self, measurement):
        return self.kalman.correct(measurement)

# Create an instance of the Kalman filter
pose_filter = PoseKalmanFilter()

# Example callback function for ArUco pose estimation
def smooth_pose_estimation(corners, ids, rvecs, tvecs):
    smoothed_poses = []

    for i in range(len(ids)):
        # Extract raw measurements (translation and rotation vectors)
        tvec = tvecs[i].flatten()
        rvec = rvecs[i].flatten()

        # Convert rotation vector to Euler angles (roll, pitch, yaw)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        euler_angles = cv2.decomposeProjectionMatrix(np.hstack((rotation_matrix, [[0], [0], [0]])))[-1]
        roll, pitch, yaw = euler_angles.flatten()

        # Construct measurement vector: [x, y, z, roll, pitch, yaw]
        measurement = np.array([tvec[0], tvec[1], tvec[2], roll, pitch, yaw], dtype=np.float32)

        # Predict the next state
        predicted = pose_filter.predict()

        # Update the Kalman filter with the current measurement
        corrected = pose_filter.correct(measurement)

        # Save the corrected pose for use
        smoothed_poses.append(corrected[:6])

    return smoothed_poses

# Example loop for pose estimation with smoothing
cap = cv2.VideoCapture(0)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        # Estimate poses
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)

        # Smooth the poses
        smoothed_poses = smooth_pose_estimation(corners, ids, rvecs, tvecs)

        # Visualize the results
        for i in range(len(ids)):
            smoothed_pose = smoothed_poses[i]
            tvec = smoothed_pose[:3]
            roll, pitch, yaw = smoothed_pose[3:]




#from functions import *

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main = {"size": (640, 360)}))

# Start the camera
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

centered = False

centre_of_frame = (int(FRAME_DIMENSIONS[0]/2),int(FRAME_DIMENSIONS[1]/2))

aruco_dictionary_name = cv2.aruco.DICT_4X4_50

def get_marker_centre(marker_id):
    centre_x1 = (int(corners[marker_id][0][0][0])+int(corners[marker_id][0][1][0]))/2
    centre_x2 = (int(corners[marker_id][0][2][0])+int(corners[marker_id][0][3][0]))/2
    centre_x = int((centre_x1+centre_x2)/2)

    centre_y1 = (int(corners[marker_id][0][0][1])+int(corners[marker_id][0][1][1]))/2
    centre_y2 = (int(corners[marker_id][0][2][1])+int(corners[marker_id][0][3][1]))/2
    centre_y = int((centre_y1+centre_y2)/2) 
    centre = (centre_x, centre_y)
    return centre
#correction for size of calibration images
aruco_marker_side_length = ARUCO_MARKER_SIZE / (1280/FRAME_DIMENSIONS[0])

# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'

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
 
# Load the camera parameters from the saved file
cv_file = cv2.FileStorage(
    camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

while True:
    # Capture frame
    frame = picam2.capture_array()
    # Operations on the frame
    frame = cv2.resize(frame, FRAME_DIMENSIONS)
    frame = cv2.flip(frame, -1)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    corners, marker_ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    if marker_ids is not None:
        
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
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera

        smoothed_poses = smooth_pose_estimation(corners, ids, rvecs, tvecs)

        for i, marker_id in enumerate(marker_ids):
       
            smoothed_pose = smoothed_poses[i]
            tvec = smoothed_pose[:3]
            roll, pitch, yaw = smoothed_pose[3:]
            cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
            
        """ 
        # Store the translation (i.e. position) information
            transform_translation_x = tvecs[i][0][0]
            transform_translation_y = tvecs[i][0][1]
            transform_translation_z = tvecs[i][0][2]
     
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
             
            roll_x_deg = math.degrees(roll_x)
            pitch_y = math.degrees(pitch_y)
            yaw_z = math.degrees(yaw_z)
            print("transform_translation_x: {}".format(transform_translation_x))
            print("transform_translation_y: {}".format(transform_translation_y))
            print("transform_translation_z: {}".format(transform_translation_z))
            print(f"roll_x in rad: {roll_x}")
            print(f"roll_x in deg: {roll_x_deg}")
        """

            #print("pitch_y: {}".format(pitch_y))
            #print("yaw_z: {}".format(yaw_z)) 
             
            # Draw the axes on the marker
            

        try:
            centre = get_marker_centre(0)
            cv2.putText(frame, ".", centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        except Exception as e:
            print(e)
            pass
        

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
picam2.stop()
cv2.destroyAllWindows()
