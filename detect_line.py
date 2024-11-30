from config import *

import numpy as np
import cv2
from simple_pid import PID

cap = cv2.VideoCapture(4)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame_orig = cap.read()
    # if frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame. Exiting ...")
        break

    frame = cv2.resize(frame_orig, FRAME_DIMENSIONS)
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
    image_draw = frame
    lines = cv2.Canny(frame_gray, 50, 150)

    try:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE ,cv2.CHAIN_APPROX_NONE)
        contour = max(contours, key = cv2.contourArea, default=0)
        cv2.drawContours(image_draw, [contour], -1, (0, 255, 0), -1)
        [vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)                         
        lefty = int((-x*vy/vx) + y)
        righty = int(((HEIGHT_OF_IMAGE-x)*vy/vx)+y)
        vy = float(vy)
        vx = float(vx)

        if 0<vy<1:
            line_angle = 180 - np.degrees(np.arctan(vy/vx))
        elif -1<vy<0:
            line_angle = np.degrees(np.arctan(np.abs(vy)/vx))
        else:
            line_angle = 90

        print(f"line_angle: {line_angle}")

        cv2.line(image_draw,(HEIGHT_OF_IMAGE-1,righty),(0,lefty),(0,255,255),5)

    except Exception as e:
        print(e)
        pass

    if len(contours)>0:
        M = cv2.moments(contour)
        if(M["m10"] !=0 and M["m01"] !=0 and M["m00"] !=0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center_of_mass = (round(cX/WIDTH_OF_IMAGE, 2), round(cY/HEIGHT_OF_IMAGE, 2))
            offset = center_of_mass[0] - 0.5
            cv2.putText(image_draw, f".", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), 10)
            cv2.putText(image_draw, f"{offset}", (cX, cY-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    else:
        pass

    # Display the resulting frame
    cv2.imshow('frame', frame_orig)
    cv2.imshow('masked_image', masked_image)
    cv2.imshow('image_draw', image_draw)
    cv2.imshow('lines', lines)
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()