import cv2
import numpy as np
from libcamera import controls, Transform
from picamera2 import Picamera2

FRAME_DIMENSIONS = (640, 360)
WIDTH_OF_IMAGE = FRAME_DIMENSIONS[0]
HEIGHT_OF_IMAGE = FRAME_DIMENSIONS[1]
CENTRE_OF_FRAME = (int(FRAME_DIMENSIONS[0]/2),int(FRAME_DIMENSIONS[1]/2))

# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'

#ARUCO dictionary used for marker detection
ARUCO_DICT_CONFIG = cv2.aruco.DICT_4X4_50

# percentage of frame for black line to be sought
LINE_SEEKING_PORTION = 20

ARUCO_MARKER_SIZE = 0.0355

# margins of error for marker alignment
MARGIN_OF_CENTER_MISALIGNMENT = 30
MARGIN_OF_DISTANCE = 3
MARGIN_OF_ANGLE = 2

DISTANCE_FROM_MARKER = 15

SPEED_STRAFE = 17
SPEED_ROLL = 14
SPEED_DRIVE = 10
SPEED_SPIN = 13

MAX_SPEED = 28
MIN_SPEED = 15

SPEED_COEF = 0.5
OFFSET_COEF = 0.33
ANGLE_COEF = 0.17

KALMAN_PROCESS_COEF = 1e-6               #Q
KALMAN_MEASUREMENT_COEF = 5e-6          #R
KALMAN_ERROR_COEF = 1

def start_camera():
	picam2 = Picamera2()
	picam2.configure(picam2.create_video_configuration(main = {"size": FRAME_DIMENSIONS}, transform = Transform(hflip=1, vflip=1)))
	picam2.start()
	picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 5.0})
	return picam2
	
def start_aruco():
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
	detectorParams = cv2.aruco.DetectorParameters()
	detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
	return detector
