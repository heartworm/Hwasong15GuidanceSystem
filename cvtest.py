import numpy as np
import cv2
from pi_stream import PiStream
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

with PiStream(resolution = resolution) as stream:
    fps = gen_fps()
    while showVideo:
        img = stream.get_frame()

        analyser.analyse(img)

        fps_str = str(round(next(fps), 2))
        cv2.putText(img, fps_str, (10, resolution[1] - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0))
        # cv2.imshow('videooutput', img)

        if cv2.waitKey(1) != -1:
            showVideo = False

cv2.destroyAllWindows()
