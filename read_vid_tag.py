import numpy as np
import cv2
import serial
import time

dimensions = (640, 480)
centre_of_frame = (int(dimensions[0]/2),int(dimensions[1]/2))
margin = 50
color_of_center = (0,0,255)

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

cap = cv2.VideoCapture(0)

while True:
    
    ret, frame = cap.read()
    frame = cv2.resize(frame, dimensions)
	# if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.rectangle(frame, (centre_of_frame[0] - margin , centre_of_frame[1] - margin), (centre_of_frame[0] + margin , centre_of_frame[1] + margin), color_of_center, 3)
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
        
        if centre_x1 > centre_of_frame[0] - margin and centre_x2 < centre_of_frame[0] + margin and centre_y1 > centre_of_frame[1] - margin and centre_y2 < centre_of_frame[1] + margin:
            color_of_center = (0,255,0)
			
        print(centre)
        
        cv2.putText(frame, ".", centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
		
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
