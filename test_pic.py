import cv2

image = cv2.imread("pics/aruco_marker3.png")

cv2.imshow("Aruco Marker", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
