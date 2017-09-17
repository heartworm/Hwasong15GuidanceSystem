#import required modules
import VREP.vrep
import pygame
import cv2
from omni import Omni
import numpy as np
from vision import ImageAnalyser
import time

print ('Program started')
VREP.vrep.simxFinish(-1) # just in case, close all opened connections

clientID = VREP.vrep.simxStart('127.0.0.1', 12345, True, True, 5000, 5) # Connect to V-REP
analyser = ImageAnalyser()

if clientID != -1:
    print('connected')
    camera = VREP.vrep.simxGetObjectHandle(clientID, "Camera1", VREP.vrep.simx_opmode_blocking)[1]
    ball = VREP.vrep.simxGetObjectHandle(clientID, "ball", VREP.vrep.simx_opmode_blocking)[1]

    while VREP.vrep.simxGetConnectionId(clientID) != -1:

        code, res, img = VREP.vrep.simxGetVisionSensorImage(clientID, camera, 0, VREP.vrep.simx_opmode_streaming)

        if code == VREP.vrep.simx_return_ok:
            img = np.array(img, dtype='uint8')
            img = np.reshape(img, list(res) + [3])
            img = np.swapaxes(img, 0, 1)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            analyser.analyse(img)
            if analyser.ballPos is not None:
                print(analyser.ballPos)
                code, actualPos = VREP.vrep.simxGetObjectPosition(clientID, ball, camera, VREP.vrep.simx_opmode_streaming)
                if code == VREP.vrep.simx_return_ok:
                    actualPos = actualPos[1:3]
                    print("Actual:", actualPos)
                    error = np.round(np.subtract(analyser.ballPos, actualPos) * 1000)
                    print("Error (mm)", error)
                time.sleep(0.5)

cv2.destroyAllWindows()
