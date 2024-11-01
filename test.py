import cv2
import cv2.aruco as aruco
import numpy as np


#IMAGE
image_orig = cv2.VideoCapture(0)
image_scaled = cv2.resize(image_orig, (300, 300))
image_grayscale = cv2.cvtColor(image_scaled, cv2.COLOR_BGR2GRAY)

while True:
    ret, frame = image_orig.read()
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
