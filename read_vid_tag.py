import numpy as np
import cv2
import serial
import time
from config import *

centre_of_frame = (int(FRAME_DIMENSIONS[0]/2),int(FRAME_DIMENSIONS[1]/2))

centered = False

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

cap = cv2.VideoCapture(0)

while True:
    if centered:
        color_of_center = (0,255,0)
    elif not centered:
        color_of_center = (0,0,255)

    ret, frame = cap.read()
    frame = cv2.resize(frame, FRAME_DIMENSIONS)
	# if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.rectangle(frame, (centre_of_frame[0] - MARGIN_OF_CENTER , centre_of_frame[1] - MARGIN_OF_CENTER), (centre_of_frame[0] + MARGIN_OF_CENTER , centre_of_frame[1] + MARGIN_OF_CENTER), color_of_center, 3)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    #print("corners:{corners}, ids:{ids}".format(corners=corners, ids=ids, rejected=rejectedImgPoints))
    
    if ids is not None:
        ids = ids.flatten()
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(255, 0, 0))
    
        centre_x1 = (int(corners[0][0][0][0])+int(corners[0][0][1][0]))/2
        centre_x2 = (int(corners[0][0][2][0])+int(corners[0][0][3][0]))/2
        centre_x = int((centre_x1+centre_x2)/2)

        centre_y1 = (int(corners[0][0][0][1])+int(corners[0][0][1][1]))/2
        centre_y2 = (int(corners[0][0][2][1])+int(corners[0][0][3][1]))/2
        centre_y = int((centre_y1+centre_y2)/2) 
        centre = (centre_x, centre_y)
        
        if centre_x > centre_of_frame[0] - MARGIN_OF_CENTER and centre_x < centre_of_frame[0] + MARGIN_OF_CENTER and centre_y > centre_of_frame[1] - MARGIN_OF_CENTER and centre_y < centre_of_frame[1] + MARGIN_OF_CENTER:
            centered = True
        else:
            centered = False

        print(centre)
        
        cv2.putText(frame, ".", centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
    else:
        centered = False
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
data_array = [0, 10, 20, 30]
cap.release()
cv2.destroyAllWindows()
