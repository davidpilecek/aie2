import numpy as np
import cv2
from picamera2 import Picamera2
from config import *
from camera_functions import *
import serial
import time
import driving_functions as dfu

from libcamera import controls, Transform
from simple_pid import PID

import os


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main = {"size": FRAME_DIMENSIONS}, transform = Transform(hflip=1, vflip=1)))
picam2.start()
frame = picam2.capture_array()
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
cv2.imwrite("/home/pi/Desktop/aie2/pics/frame.jpg", frame)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 360))


arduino_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
arduino_port.reset_input_buffer()
time.sleep(2)

while True:
    # Capture frame
    frame = picam2.capture_array()
    # Operations on the frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_blurred = cv2.GaussianBlur(frame, (21, 21), 0)
    frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2GRAY)
  
    mask = cv2.adaptiveThreshold(
        frame_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 191, 20
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
    frame_draw1 = frame

    try:
        contours, hierarchy = cv2.findContours(masked_image, cv2.RETR_TREE ,cv2.CHAIN_APPROX_NONE)
        contour = max(contours, key = cv2.contourArea, default=0)
        cv2.drawContours(frame_draw1, [contour], -1, (0, 255, 0), -1)
        [vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)                         
        lefty = int((-x*vy/vx) + y)
        righty = int(((HEIGHT_OF_IMAGE-x)*vy/vx)+y)
        vy = float(vy)
        vx = float(vx)
        
        vx = abs(vx-1)
        
        if vy<0:
            vy = abs(vy+1)
        elif vy>0:
            vy = vy-1
        else:
            line_angle = 0

        line_angle = np.arctan(vy/vx) * (2/np.pi)
        line_angle = round(line_angle, 2)
        print(f"line_angle: {line_angle}")
        
    except Exception as e:
        print(e)
        dfu.stop_all(arduino_port)
        pass

    if len(contours)>0:
        M = cv2.moments(contour)
        if(M["m10"] !=0 and M["m01"] !=0 and M["m00"] !=0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center_of_mass = (round(cX/WIDTH_OF_IMAGE, 2), round(cY/HEIGHT_OF_IMAGE, 2))
            offset = round(2 * (center_of_mass[0]-0.5), 2)
            cv2.putText(frame_draw1, f".", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), 10)
            cv2.putText(frame_draw1, f"{offset}", (cX-50, cY-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            cv2.line(frame_draw1,(HEIGHT_OF_IMAGE,righty),(cX,cY),(0,255,255),5)

    else:
        dfu.stop_all(arduino_port)
        print("line lost")
        pass
    
    # Display the resulting frame
    cv2.imshow('frame', frame)
    out.write(frame)
    
    if cv2.waitKey(1) == ord('q'):

        cv2.imwrite("/home/pi/Desktop/aie2/pics/blur.jpg", frame_blurred)
        cv2.imwrite("/home/pi/Desktop/aie2/pics/gray.jpg", frame_gray)
        cv2.imwrite("/home/pi/Desktop/aie2/pics/masked.jpg", masked_image)
        cv2.imwrite("/home/pi/Desktop/aie2/pics/draw0.jpg", mask)
      #  cv2.imwrite("/home/pi/Desktop/aie2/pics/draw1.jpg", frame_draw1)
        #cv2.imwrite("/home/pi/Desktop/aie2/pics/draw2.jpg", frame_draw1)
        cv2.imwrite("/home/pi/Desktop/aie2/pics/draw3.jpg", frame_draw1)
        break
      



    
# When everything done, release the capture
picam2.stop()
cv2.destroyAllWindows()
dfu.stop_all(arduino_port)
