import numpy as np
import cv2
from picamera2 import Picamera2
from config import *

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)


# Capture frame
frame = cv2.imread("/home/pi/Desktop/aie2/pics/blur.jpg")
# Operations on the frame
frame = cv2.resize(frame, FRAME_DIMENSIONS)

#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
frame_blurred = cv2.GaussianBlur(frame, (21, 21), 0)
    
    
mask = cv2.adaptiveThreshold(
    frame_blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 191, 20
)
crop_selection = 100 / (100 - LINE_SEEKING_PORTION)
height_1 = HEIGHT_OF_IMAGE / crop_selection
vertices = [
    (0, height_1),
    (0, HEIGHT_OF_IMAGE),
    (WIDTH_OF_IMAGE, HEIGHT_OF_IMAGE),
    (WIDTH_OF_IMAGE, height_1)
]

vertices = np.array([vertices], np.int32)
mask_black = np.zeros_like(mask)
match_mask_color = [255, 255, 255]
cv2.fillPoly(mask_black, vertices, match_mask_color)
masked_image = cv2.bitwise_and(mask, mask_black)
frame_draw = frame	
        
        
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

