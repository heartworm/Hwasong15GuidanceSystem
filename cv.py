#import required modules
import vrep
import pygame
import cv2
from omni import Omni
import numpy as np
from vision import ImageAnalyser
import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 12345, True, True, 5000, 5) # Connect to V-REP
analyser = ImageAnalyser()

if clientID != -1:
    print('connected')
    camera = vrep.simxGetObjectHandle(clientID, "Camera1", vrep.simx_opmode_blocking)[1]
    ball = vrep.simxGetObjectHandle(clientID, "ball", vrep.simx_opmode_blocking)[1]

    while vrep.simxGetConnectionId(clientID) != -1:

        code, res, img = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)

        if code == vrep.simx_return_ok:
            img = np.array(img, dtype='uint8')
            img = np.reshape(img, list(res) + [3])
            img = np.swapaxes(img, 0, 1)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            analyser.analyse(img)

            print("Ball: ", analyser.ballPos)
            print("Goal: ", analyser.goalPos)
            print("Obstacle: ", analyser.obstaclePoses)
            print("Wall: ", analyser.wallPoses)

            

            # if analyser.ballPos is not None:
            #     print(analyser.ballPos)
            #     code, actualPos = vrep.simxGetObjectPosition(clientID, ball, camera, vrep.simx_opmode_streaming)
            #     # if code == vrep.simx_return_ok:
            #     #     actualPos = actualPos[1:3]
            #     #     print("Actual:", actualPos)
            #     #     error = np.round(np.subtract(analyser.ballPos[0:2], actualPos) * 1000)
            #     #     print("Error (mm)", error)
            #     time.sleep(0.5)

cv2.destroyAllWindows()
