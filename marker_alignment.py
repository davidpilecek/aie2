import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
import serial
import time
import driving_functions as dfu
from scipy.spatial.transform import Rotation as R
import math
#from functions import *

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())

# Start the camera
picam2.start()

aligned_translation = False
aligned_rotation = False
aligned_distance = False

marker_state_holder = False            #holder for if marker is lost
seeing_marker = False
marker_lost = True

last_marker_position = 0             # -1 = left, 1 = right 

centre_of_frame = (int(FRAME_DIMENSIONS[0]/2),int(FRAME_DIMENSIONS[1]/2))

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


while True:
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
    cv2.line(frame, (centre_of_frame[0] - MARGIN_OF_CENTER , 0), (centre_of_frame[0] - MARGIN_OF_CENTER , FRAME_DIMENSIONS[1]), color_of_center, 3)
    cv2.line(frame, (centre_of_frame[0] + MARGIN_OF_CENTER , 0), (centre_of_frame[0] + MARGIN_OF_CENTER , FRAME_DIMENSIONS[1]), color_of_center, 3)
    
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
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
        for i, marker_id in enumerate(marker_ids):
       
        # Store the translation (i.e. position) information
            transform_translation_x = tvecs[i][0][0] 
            transform_translation_y = tvecs[i][0][1]
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
            pitch_y = math.degrees(pitch_y)
            yaw_z = math.degrees(yaw_z)
            print("transform_translation_x: {}".format(transform_translation_x))
            print("transform_translation_z: {}".format(transform_translation_z))
            print(f"roll_x in deg: {roll_x_deg}")
            #print("pitch_y: {}".format(pitch_y))
            #print("yaw_z: {}".format(yaw_z))
            print()
             
            # Draw the axes on the marker
            cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
        try:            
            centre_x, centre_y = get_marker_centre(0)
            cv2.putText(frame, ".", (centre_x, centre_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            if centre_x < centre_of_frame[0] - MARGIN_OF_CENTER:
                last_marker_position = -1
                
                dfu.slide_left(SPEED_SLIDE, ser)
                aligned_translation = False
            elif centre_x > centre_of_frame[0] + MARGIN_OF_CENTER:
                last_marker_position = 1
                
                dfu.slide_right(SPEED_SLIDE, ser)
                aligned_translation = False
            else:
                last_marker_position = 0
                aligned_translation = True

            if 90 < roll_x_deg < 180 - MARGIN_OF_ANGLE:
                dfu.roll_left(SPEED_ROLL, ser)
                aligned_rotation = False
            elif -180 + MARGIN_OF_ANGLE < roll_x_deg < -90:
                dfu.roll_right(SPEED_ROLL, ser)
                aligned_rotation = False
            else:
                aligned_rotation = True
                
            if aligned_translation and aligned_rotation:

                if transform_translation_z > MARGIN_OF_DISTANCE:
                    dfu.drive_straight(SPEED_DRIVE, ser)
                    aligned_distance = False
                    
                else:
                    aligned_distance = True
                    dfu.stop_all(ser)
    
        except Exception as e:
            print(e)
            pass
    else:
        dfu.stop_all(ser)
        marker_lost = True 
        
    if marker_lost:
        if last_marker_position == 1:
            dfu.slide_right(SPEED_SLIDE, ser)
            print("going right")
        elif last_marker_position == -1:
            dfu.slide_left(SPEED_SLIDE, ser)
            print("going left")
        elif last_marker_position == 0:
            dfu.spin_right(SPEED_SPIN, ser)
    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
dfu.stop_all(ser)
picam2.stop()
cv2.destroyAllWindows()
