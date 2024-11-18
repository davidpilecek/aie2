import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
from functions import *

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())

# Start the camera
picam2.start()

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

def get_marker_centre(marker_id):
    centre_x1 = (int(corners[marker_id][0][0][0])+int(corners[marker_id][0][1][0]))/2
    centre_x2 = (int(corners[marker_id][0][2][0])+int(corners[marker_id][0][3][0]))/2
    centre_x = int((centre_x1+centre_x2)/2)

    centre_y1 = (int(corners[marker_id][0][0][1])+int(corners[marker_id][0][1][1]))/2
    centre_y2 = (int(corners[marker_id][0][2][1])+int(corners[marker_id][0][3][1]))/2
    centre_y = int((centre_y1+centre_y2)/2) 
    centre = (centre_x, centre_y)
    return centre
    

while True:
    # Capture frame
    frame = picam2.capture_array()
    # Operations on the frame
    frame = cv2.resize(frame, FRAME_DIMENSIONS)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    
    
    if ids is not None:
        ids = ids.flatten()
        print(f"corners:{corners}, \n, len: {corners.__sizeof__()} ids:{ids}\n")
     #   print(f"corners:{corners}, \n, ids:{ids}\n")
      #  print(f"corners:{corners}, \n, ids:{ids}\n")
        
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(255, 0, 0))
    
        centre_x1 = (int(corners[0][0][0][0])+int(corners[0][0][1][0]))/2
        centre_x2 = (int(corners[0][0][2][0])+int(corners[0][0][3][0]))/2
        centre_x = int((centre_x1+centre_x2)/2)

        centre_y1 = (int(corners[0][0][0][1])+int(corners[0][0][1][1]))/2
        centre_y2 = (int(corners[0][0][2][1])+int(corners[0][0][3][1]))/2
        centre_y = int((centre_y1+centre_y2)/2) 
        centre = (centre_x, centre_y)
        
        
        cv2.putText(frame, ".", centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
picam2.stop()
cv2.destroyAllWindows()
