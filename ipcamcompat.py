import numpy as np
import cv2
import urllib2
import test

LINK = "http://192.168.69.2:8080/video"

stream = urllib2.urlopen(LINK)
buf = ''
showVideo = True

cv2.namedWindow('videooutput')
cv2.namedWindow('ballfinder')

while showVideo:
    buf += stream.read(1024)
    a = buf.rfind('\xff\xd8')
    b = buf.rfind('\xff\xd9')


    if a != -1 and b != -1 and b > a:
        jpg = buf[a:b+2]
        buf = buf[b+2:]

        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), 1)
        img = test.denoise(img)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        cv2.imshow('videooutput', img)
        cv2.imshow('ballfinder', test.getBallMaskHSV(hsv))
        if cv2.waitKey(1) != -1:
            showVideo = False

cv2.destroyAllWindows()
stream.close()
	
