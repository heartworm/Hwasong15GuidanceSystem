import numpy as np
import cv2
import urllib2
import requests
import test

LINK = "http://192.168.69.2:8080/shot.jpg"
buf = ''
showVideo = True


def show_val(event,x,y,flags,param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    pixel = hsv[y,x]
    print("HSV at {}, {}: ({}, {}, {})".format(x, y, pixel[0], pixel[1], pixel[2]))


cv2.namedWindow('videooutput')
cv2.setMouseCallback('videooutput', show_val)

cv2.namedWindow('ballfinder')

while showVideo:
    r = requests.get(LINK)
    r.raise_for_status()
    
    img = cv2.imdecode(np.fromstring(r.content, dtype=np.uint8), 1)
    img = test.denoise(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        


    ballMask = test.getBallMaskHSV(hsv)
    
    cv2.imshow('ballfinder', ballMask)

    keypoints = test.getBallBlobKeypoints(255 - ballMask)
    img = cv2.drawKeypoints(img, keypoints, np.array([]), (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
        
    cv2.imshow('videooutput', img)    


    if cv2.waitKey(1) != -1:
        showVideo = False

cv2.destroyAllWindows()
