import numpy as np
import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())

# Start the camera
picam2.start()

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)



while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()
 
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    #print("corners:{corners}, ids:{ids}".format(corners=corners, ids=ids, rejected=rejectedImgPoints))
    
    if ids is not None:
        ids = ids.flatten()
        #frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(255, 0, 0))
    
        centre_x1 = (int(corners[0][0][0][0])+int(corners[0][0][1][0]))/2
        centre_x2 = (int(corners[0][0][2][0])+int(corners[0][0][3][0]))/2
        centre_x = int((centre_x1+centre_x2)/2)

        centre_y1 = (int(corners[0][0][0][1])+int(corners[0][0][1][1]))/2
        centre_y2 = (int(corners[0][0][2][1])+int(corners[0][0][3][1]))/2
        centre_y = int((centre_y1+centre_y2)/2) 
        centre = (centre_x, centre_y)
        print(centre)
        
        cv2.putText(frame, ".", centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
picam2.stop()
cv2.destroyAllWindows()
