import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
from libcamera import controls, Transform
from camera_functions import *

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(transform = Transform(hflip=1, vflip=1)))
picam2.start()

while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB )
    

    # Display the resulting frame
    cv2.imshow('frame', frame)
    # ~ cv2.imshow('gray', gray)
    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
