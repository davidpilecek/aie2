import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
from libcamera import controls, Transform


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main = {"size": FRAME_DIMENSIONS}, transform = Transform(hflip=1, vflip=1)))
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 5.0})

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
