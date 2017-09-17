import numpy as np
import cv2
from pi_stream import PiStream
import test
import timeit

showVideo = True

def show_val(event,x,y,flags,param):

    if event != cv2.EVENT_LBUTTONDOWN:
        return
    pixel = hsv[y,x]
    print("HSV at {}, {}: ({}, {}, {})".format(x, y, pixel[0], pixel[1], pixel[2]))


cv2.namedWindow('videooutput')
cv2.setMouseCallback('videooutput', show_val)

cv2.namedWindow('ballfinder')

fps_lasttime = timeit.default_timer()
fps = 0
alpha = 0.95

def get_fps():
    global fps, fps_lasttime
    newtime = timeit.default_timer()
    dtime = newtime - fps_lasttime
    newfps = 1 / dtime
    fps = newfps * (1 - alpha) + fps * alpha
    fps_lasttime = newtime
    return fps


resolution = (160,120)

with PiStream(resolution = resolution) as stream:
    while showVideo:
        img = stream.get_frame()
        denoised = test.denoise(img)

        hsv = cv2.cvtColor(denoised, cv2.COLOR_RGB2HSV)


        ballMask = test.getBallMaskHSV(hsv)

        cv2.imshow('ballfinder', ballMask)

        keypoints = test.getBallBlobKeypoints(255 - ballMask)
        img = cv2.drawKeypoints(img, keypoints, np.array([]), (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        fps_str = str(round(get_fps(), 2))
        cv2.putText(img, fps_str, (10, resolution[1] - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0))


        cv2.imshow('videooutput', img)


        if cv2.waitKey(1) != -1:
            showVideo = False

cv2.destroyAllWindows()
