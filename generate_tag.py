import cv2
import cv2.aruco as aruco
import numpy as np
from config import *

# Define the dictionary (size and ID for the marker)
# Common dictionaries: DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_7X7_1000
#aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_CONFIG)

# Specify the ID and size of the marker
marker_id = 3  # ID can range based on the dictionary chosen
marker_size = 300  # Size of the marker in pixels

# Generate the marker
marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)


final_image = cv2.copyMakeBorder(marker_image, 25, 25, 25, 25 , cv2.BORDER_CONSTANT, value=[255, 255, 255])

# Save the marker as an image file
cv2.imwrite("pics/aruco_marker3.png", final_image)

# Display the generated marker (optional)
cv2.imshow("Aruco Marker", marker_image)
cv2.imshow("Final Image", final_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
