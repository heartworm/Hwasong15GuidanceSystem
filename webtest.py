import numpy as np
import cv2
import timeit
from webcam_stream import WebcamStream
from vision import ImageAnalyser
import ai
import yaml

showVideo = True
with open('config.yaml') as yamlfile:
    analyser = ImageAnalyser(yaml.load(yamlfile))

try:
    with WebcamStream() as cam:
        for frame in cam.frames():
            analyser.analyse(frame)
            ai.state_controller(analyser.ballPos, analyser.obstaclePoses, analyser.goalPos, analyser.wallPoses)
except KeyboardInterrupt:
    print("quitting.")
cv2.destroyAllWindows()
