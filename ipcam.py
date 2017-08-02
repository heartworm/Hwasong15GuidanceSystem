import numpy as np
import cv2

LINK = "http://192.168.69.2:8080/video"

stream = cv2.VideoCapture()
stream.open(LINK)

if not stream.isOpened():
	print("Failed to open stream!")
	exit(1)
	
cv2.namedWindow('videooutput')
showVideo = True
	
while showVideo and stream.isOpened():
	grabbed, frame = stream.read()
	if grabbed:
		cv2.imshow('videooutput', frame)
		key = cv2.waitKey(1)
		if key != 255:
			showVideo = False
			
cv2.destroyAllWindows()
stream.release()
	