import cv2
import cv2.aruco as aruco

# Define the dictionary (size and ID for the marker)
# Common dictionaries: DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_7X7_1000
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Specify the ID and size of the marker
marker_id = 0  # ID can range based on the dictionary chosen
marker_size = 200  # Size of the marker in pixels

# Generate the marker
marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Save the marker as an image file
cv2.imwrite("pics/aruco_marker.png", marker_image)

# Display the generated marker (optional)
cv2.imshow("Aruco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
