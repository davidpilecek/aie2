import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
import functions as myf	

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)


# Capture frame
frame = cv2.imread("/home/pi/Desktop/aie2/pics/aruco_5x.png")
# Operations on the frame
frame = cv2.resize(frame, FRAME_DIMENSIONS)
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB )
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
corners, ids, rejectedImgPoints = detector.detectMarkers(gray)


if ids is not None:
    ids = ids.flatten()
    print(f"corners:{corners[1]}, \n, len: {len(corners)} ids:{ids[1]}\n")
     #   print(f"corners:{corners}, \n, ids:{ids}\n")
      #  print(f"corners:{corners}, \n, ids:{ids}\n")
        
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(255, 0, 0))
    
    for marker_id in range(len(corners)):
        centre = myf.get_marker_centre(marker_id)
        cv2.putText(frame, ".", centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

# Display the resulting frame
cv2.imshow('frame', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()

