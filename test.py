import numpy as np
import cv2


def show_val(event,x,y,flags,param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    pixel = hsv[y,x]
    print("HSV at {}, {}: ({}, {}, {})".format(x, y, pixel[0], pixel[1], pixel[2]))

    
def getBallMaskHSV(img):
    ballLower = np.array([105, 150, 50])
    ballUpper = np.array([125, 250, 255])
    ballMask = cv2.inRange(img, ballLower, ballUpper)
    ballMaskEroded = cv2.erode(ballMask, None)
    return ballMaskEroded
    
def getWallMaskHSV(img):
    wallLower = np.array([0, 0, 0])
    wallUpper = np.array([255, 50, 255])
    wallMask = cv2.inRange(img, wallLower, wallUpper)
    return wallMask

def getBallBlobKeypoints(mask):
    ballDetectorParams = cv2.SimpleBlobDetector_Params()
    ballDetectorParams.filterByArea = False
    ballDetectorParams.filterByConvexity = False
    ballDetectorParams.filterByCircularity = False
    ballDetectorParams.filterByInertia = False
    ballDetectorParams.minThreshold = 0
    ballDetectorParams.maxThreshold = 255
    ballDetectorParams.minCircularity = 0.6
    ballDetectorParams.minArea = 100
    
    ballDetector = cv2.SimpleBlobDetector_create(ballDetectorParams)
    keypoints = ballDetector.detect(mask)
    return keypoints

    
    
def denoise(img):
    blurredImg = cv2.medianBlur(img, 9)
    return blurredImg
    
img = cv2.imread("soccerball.jpg")
denoised = denoise(img)
hsv = cv2.cvtColor(denoised, cv2.COLOR_RGB2HSV)

ballMask = getBallMaskHSV(hsv)
wallMask = getWallMaskHSV(hsv)

ballKeypoints = getBallBlobKeypoints(255 - ballMask)
keypointsImg = cv2.drawKeypoints(img, ballKeypoints, np.array([]), (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


cv2.namedWindow('image')
cv2.setMouseCallback('image', show_val)

cv2.imshow('image', denoised)
cv2.imshow('balls', ballMask)
cv2.imshow('walls', wallMask)
cv2.imshow('blobs', keypointsImg)
cv2.waitKey(0)
cv2.destroyAllWindows()


