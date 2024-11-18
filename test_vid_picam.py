import cv2
from picamera2 import Picamera2
from config import *


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())

# Start the camera
picam2.start()

while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()
    frame = cv2.resize(frame, FRAME_DIMENSIONS)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB )
    
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    cv2.imshow('gray', gray)
    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
