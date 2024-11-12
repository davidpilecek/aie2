import cv2
import numpy as np

ARUCO_DICT_CONFIG = cv2.aruco.DICT_4X4_50

FRAME_DIMENSIONS = (640, 480)
WIDTH_OF_IMAGE = FRAME_DIMENSIONS[0]
HEIGHT_OF_IMAGE = FRAME_DIMENSIONS[1]

#percentage of frame for black line to be sought
LINE_SEEKING_PORTION = 60

MARGIN_OF_CENTER = 50

centered = False


