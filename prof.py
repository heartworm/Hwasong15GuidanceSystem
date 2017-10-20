from webcam_stream import WebcamStream
from vision import ImageAnalyser
import yaml
import ai
from drive import Drive
import math


with open('config.yaml') as yamlfile:
    config = yaml.load(yamlfile)

# ia = ImageAnalyser(config)
#
# with WebcamStream() as stream:
#     frames = stream.frames()
#     for i in range(10):
#         print(i)
#         frame = next(frames)
#         ia.analyse(frame)
#         ai_heading, ai_velocity = ai.state_controller(ia.ballPos, ia.obstaclePoses, ia.goalPos, ia.wallPoses)
#

drive = Drive(config)
