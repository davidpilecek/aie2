import cv2
import numpy as np
from libcamera import controls, Transform
from picamera2 import Picamera2
from config import *

def start_camera():
	picam2 = Picamera2()
	picam2.configure(picam2.create_video_configuration(main = {"size": FRAME_DIMENSIONS}, transform = Transform(hflip=1, vflip=1)))
	picam2.start()
	picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 5.0})
	return picam2
	
def start_aruco():
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
	detectorParams = cv2.aruco.DetectorParameters()
	detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

	# Load the camera parameters from the saved file
	cv_file = cv2.FileStorage(
    	camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
	mtx = cv_file.getNode('K').mat()
	dst = cv_file.getNode('D').mat()
	cv_file.release()
	return detector, mtx, dst

def get_marker_centre(marker_id, corners):
    centre_x1 = (int(corners[marker_id][0][0][0])+int(corners[marker_id][0][1][0]))/2
    centre_x2 = (int(corners[marker_id][0][2][0])+int(corners[marker_id][0][3][0]))/2
    centre_x = int((centre_x1+centre_x2)/2)

    centre_y1 = (int(corners[marker_id][0][0][1])+int(corners[marker_id][0][1][1]))/2
    centre_y2 = (int(corners[marker_id][0][2][1])+int(corners[marker_id][0][3][1]))/2
    centre_y = int((centre_y1+centre_y2)/2) 
    
    return centre_x, centre_y

def create_mask(frame, frame_blurred, mask):

    mask = cv2.adaptiveThreshold(
        frame_blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 191, 20
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
    frame_draw = frame	

    return masked_image, frame_draw

def get_contours(masked_image, frame_draw):
    contours, hierarchy = cv2.findContours(masked_image, cv2.RETR_TREE ,cv2.CHAIN_APPROX_NONE)
    max_contour = max(contours, key = cv2.contourArea, default=0)
    cv2.drawContours(frame_draw, [max_contour], -1, (0, 255, 0), -1)
    return contours, max_contour, frame_draw

def get_line_angle(contour, frame_draw):

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

    cv2.line(frame_draw,(HEIGHT_OF_IMAGE-1,righty),(0,lefty),(0,255,255),5)

    return line_angle, frame_draw

def get_offset(contour, frame_draw):
    M = cv2.moments(contour)
    if(M["m10"] !=0 and M["m01"] !=0 and M["m00"] !=0):
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        center_of_mass = (round(cX/WIDTH_OF_IMAGE, 2), round(cY/HEIGHT_OF_IMAGE, 2))
        offset = round(center_of_mass[0] - 0.5, 2)
        cv2.putText(frame_draw, f".", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 0), 10)
        cv2.putText(frame_draw, f"{offset}", (cX, cY-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)


    return offset, frame_draw