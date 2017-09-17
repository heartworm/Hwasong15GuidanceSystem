#import required modules
import vrep
import pygame
import cv2
from omni import Omni
import numpy as np

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1',12345,True,True,5000,5) # Connect to V-REP

pygame.init()
pygame.display.set_mode((100,100))
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
joysticks = [joystick for joystick in joysticks if joystick.get_name() == 'Controller (Xbox One For Windows)']

if len(joysticks) > 0:
    joystick = joysticks[0]
else:
    joystick = None

omni = Omni()

cameraStreaming = False

np.set_printoptions(threshold=np.nan)

if clientID != -1:
    print('connected')
    vrep.simxSynchronous(clientID, True)
    wheels = [vrep.simxGetObjectHandle(clientID, "Wheel1", vrep.simx_opmode_blocking)[1],
              vrep.simxGetObjectHandle(clientID, "Wheel2", vrep.simx_opmode_blocking)[1],
              vrep.simxGetObjectHandle(clientID, "Wheel3", vrep.simx_opmode_blocking)[1]]
    camera = vrep.simxGetObjectHandle(clientID, "Camera1", vrep.simx_opmode_blocking)[1]

    while vrep.simxGetConnectionId(clientID) != -1:

        code, res, img = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)

        if code == vrep.simx_return_ok:
            img = np.array(img, dtype='uint8')
            img = np.reshape(img, list(res) + [3])
            img = np.swapaxes(img, 0, 1)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            cv2.imshow('img', img)
            cv2.waitKey(1)

        if joystick is not None:
            pygame.event.pump()
            vec = [-joystick.get_axis(0), joystick.get_axis(1)]
            if np.linalg.norm(vec) < 0.1:
                vec = [0, 0]
            speeds = omni.get_speeds(vec, polar=False)
            degPerSec = 5
            for i in range(len(wheels)):
                vrep.simxSetJointTargetVelocity(clientID, wheels[i], degPerSec * speeds[i], vrep.simx_opmode_oneshot)

        vrep.simxSynchronousTrigger(clientID)
cv2.destroyAllWindows()
