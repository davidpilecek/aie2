import cv2
import cv2.aruco as aruco
import numpy as np
from config import *

#IMAGE
image_orig = cv2.imread(r"pics\aruco_marker.png")
image_scaled = cv2.resize(image_orig, (300, 300))
image_grayscale = cv2.cvtColor(image_scaled, cv2.COLOR_BGR2GRAY)

#ARUCO
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_CONFIG)
detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)


corners, ids, rejectedImgPoints = detector.detectMarkers(image_orig)
print("corners:{corners}, ids:{ids}".format(corners=corners, ids=ids, rejected=rejectedImgPoints))


if ids is not None:
    ids = ids.flatten()
    final_image = cv2.aruco.drawDetectedMarkers(image_orig, corners, ids, borderColor=(255, 0, 0))
  
    centre_x1 = (int(corners[0][0][0][0])+int(corners[0][0][1][0]))/2
    centre_x2 = (int(corners[0][0][2][0])+int(corners[0][0][3][0]))/2
    centre_x = int((centre_x1+centre_x2)/2)

    centre_y1 = (int(corners[0][0][0][1])+int(corners[0][0][1][1]))/2
    centre_y2 = (int(corners[0][0][2][1])+int(corners[0][0][3][1]))/2
    centre_y = int((centre_y1+centre_y2)/2) 
    print(centre_x)
    print(centre_y)

    cv2.putText(final_image, ".", (centre_x, centre_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

# Display the generated marker (optional)
cv2.imshow("Aruco Marker", final_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
