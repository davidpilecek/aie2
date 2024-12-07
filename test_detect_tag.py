import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
from scipy.spatial.transform import Rotation as R
import math # Math library
from libcamera import controls
import matplotlib
import matplotlib.pyplot as plt
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


def smooth_pose_estimation(corners, ids, rvecs, tvecs, centre):
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

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main = {"size": FRAME_DIMENSIONS}, transform = Transform(hflip=1, vflip=1)))
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 5.0})

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
 
# Load the camera parameters from the saved file
cv_file = cv2.FileStorage(
    camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
mtx = cv_file.getNode('K').mat()
dst = cv_file.getNode('D').mat()
cv_file.release()

yaw_corrected = []
yaw_real = []

trans_corrected = []
trans_real = []

while True:
    print("\n")
    # Capture frame
    frame = picam2.capture_array()
    # Operations on the frame
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
        centre = get_marker_centre(0)
        smoothed_poses, real_yaw = smooth_pose_estimation(corners, marker_ids, rvecs, tvecs, centre)
        

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
            
            print(f"roll deg: {roll_deg:.2f}")
            print(f"pitch deg: {pitch_deg:.2f}")
            print(f"yaw deg: {yaw_deg:.2f}")
            
            cv2.putText(frame, f"roll deg: {roll_deg:.2f}", (0, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"pitch deg: {pitch_deg:.2f}", (0, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"yaw deg: {yaw_deg:.2f}", (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            #distance from tag in cm
            transform_translation_x = tvec[0] * 100
            transform_translation_z = tvec[1] * 100
            transform_translation_y = tvec[2] * 100
            cv2.putText(frame, f"translation y: {transform_translation_y:.2f}", (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"translation z: {transform_translation_z:.2f}", (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"translation x: {transform_translation_x:.2f}", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            
            #draw_array = np.array([roll_deg, pitch_deg, yaw_deg], dtype = np.float32)
            print(rvecs)
            #print(draw_array)
            cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
            
            #print(f"Position: {transform_translation_z} | Orientation (deg): Roll={roll_deg}, Pitch={pitch_deg}, Yaw={yaw_deg}")
    
        # ~ try:

        # ~ except Exception as e:
            # ~ print(e)
            # ~ pass
        
    # Display the resulting frame
    cv2.imshow('frame', frame)
    out.write(frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
picam2.stop()
cv2.destroyAllWindows()

plt.figure(figsize=(8, 6))
# ~ plt.plot(high_boundary)
# ~ plt.plot(low_boundary)
# ~ plt.plot(yaw_corrected, label="corrected yaw")
# ~ plt.plot(yaw_real, label="real yaw")

# ~ plt.plot(trans_corrected, label="corrected translation")
# ~ plt.plot(trans_real, label="real translation")

# ~ plt.xlabel("Frame")
# ~ plt.ylabel("Angle")
# ~ plt.legend()
# ~ plt.grid()
# ~ plt.savefig("trans_kalman_graph.png")
# ~ plt.savefig("aruco__kalman_graph.png")
