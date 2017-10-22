import vrep
import cv2
import numpy as np
from vision import ImageAnalyser
from ai import AI
import yaml
import math

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
deltaT = 0.05
clientID = vrep.simxStart('127.0.0.1', 12345, True, True, 5000, 5) # Connect to V-REP


with open('config.yaml') as yamlfile:
    yamlconf = (yaml.load(yamlfile))


analyser = ImageAnalyser(yamlconf)
navigation= AI(yamlconf)

if clientID != -1:
    print('connected')
    camera = vrep.simxGetObjectHandle(clientID, "Camera1", vrep.simx_opmode_blocking)[1]
    ball = vrep.simxGetObjectHandle(clientID, "ball", vrep.simx_opmode_blocking)[1]
    robot = vrep.simxGetObjectHandle(clientID,"Robot",vrep.simx_opmode_blocking)[1]
    table = vrep.simxGetObjectHandle(clientID,"Table",vrep.simx_opmode_blocking)[1]
    havelatched = 0

    while vrep.simxGetConnectionId(clientID) != -1:

        code, res, img = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_streaming)

        if code == vrep.simx_return_ok:
            img = np.array(img, dtype='uint8')
            img = np.reshape(img, list(res) + [3])
            img = np.swapaxes(img, 0, 1)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            analyser.analyse(img)
            alpha,beta,gamma = vrep.simxGetObjectOrientation(clientID, robot,-1, vrep.simx_opmode_blocking)[1]
            print("Rotation",math.degrees(gamma))
            rotation,velocity,state = navigation.state_controller(analyser.ballPos, analyser.obstaclePoses, analyser.bluePos, analyser.wallPoses)
            #print("cunt")
            #print("MOVEMENT:",movement)
            to_rotate = [alpha,beta,gamma + (deltaT * rotation)]
            #print("to_rotate:",to_rotate)
            vrep.simxSetObjectOrientation(clientID,robot,-1,to_rotate, vrep.simx_opmode_oneshot)
            vrep.simxSetObjectPosition(clientID,robot,robot,[0,-0.02 * velocity,0], vrep.simx_opmode_oneshot)
            if (state == 2):
                vrep.simxSetObjectPosition(clientID, ball, robot, (0,-0.2,0), vrep.simx_opmode_oneshot)
           
            #vrep.simxSetObjectPosition(clientID,robot,-1,objectPosition,vrep.simx_opmode_oneshot);


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
