import numpy as np
import cv2

img = cv2.imread("soccerball.jpg")
gImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
oImg = cv2.medianBlur(gImg, 9)
cv2.imshow('image', oImg)
cv2.waitKey(0)
cv2.destroyAllWindows()