import numpy as np
import cv2
import timeit
from vision import ImageAnalyser

showVideo = True
analyser = ImageAnalyser()

cv2.namedWindow('videooutput')

def gen_fps():
    fps_lasttime = timeit.default_timer()
    fps = 0
    alpha = 0.95
    while True:
        newtime = timeit.default_timer()
        dtime = newtime - fps_lasttime
        newfps = 1 / dtime
        fps = newfps * (1 - alpha) + fps * alpha
        fps_lasttime = newtime
        yield fps

resolution = (320,240)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
fps = gen_fps()
try:
    while showVideo and cap.isOpened():
        ret, img = cap.read()
        if ret:
            fps_str = str(round(next(fps), 2))
            cv2.putText(img, fps_str, (10, resolution[1] - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0))
            cv2.imshow('videooutput', img)
            analyser.analyse(img)
except KeyboardInterrupt:
    print("quitting.")
cap.release()
cv2.destroyAllWindows()
