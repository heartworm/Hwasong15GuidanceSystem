import cv2
import numpy as np
import math

class ImageAnalyser:
    def __init__(self):
        self.fov = np.array((math.radians(75), math.radians(50)))
        # self.fov = np.array((math.radians(62.2), math.radians(48.8)))
        self.tilt = math.radians(90 - 0)
        self.camHeight = 0.215
        self.res = np.array((-1,-1))
        self.ballProps = {
            "dimensions": (0.0427, 0.0427)
        }
        self.obstacleProps = {
            "dimensions": (0.18, 0.225)
        }

        self.ballPos = None

    def analyse(self, img):
        self.res = np.array(np.shape(img)[1::-1], dtype='float')
        denoised = self.denoiseRaw(img)

        hsv = cv2.cvtColor(denoised, cv2.COLOR_BGR2HSV)

        ballMask = self.ballThreshold(hsv)
        ballContours = self.findContours(np.array(ballMask))
        if ballContours is not None:
            areaGetter = lambda info: info["area"]
            ballInfos = sorted([self.contourInfo(contour) for contour in ballContours], key=areaGetter, reverse=True)
            ballInfo = ballInfos[0]
            # for ballInfo in ballInfos:
            self.ballPos = self.realCoordinates(ballInfo, self.ballProps)
            self.drawContourInfo(denoised, ballInfo, self.ballPos)
                # break

        cv2.imshow('ball', ballMask)

        # wallMask = self.wallThreshold(hsv)
        # grassMask = self.grassThreshold(hsv)

        obstacleMask = self.obstacleThreshold(hsv)
        obstacleContours = self.findContours(np.array(obstacleMask))
        if obstacleContours is not None:
            for obstacleContour in obstacleContours:
                cv2.drawContours(denoised, obstacleContours, -1, (255,0,0))
                obstacleInfo = self.contourInfo(obstacleContour)
                self.obstaclePos = self.realCoordinates(obstacleInfo, self.obstacleProps)
                # self.drawContourInfo(denoised, obstacleInfo, self.obstaclePos)
        cv2.imshow('obstacle', obstacleMask)
        # cv2.imshow('wall', wallMask)
        # cv2.imshow('grass', grassMask)
        cv2.imshow('denoised', denoised)
        cv2.waitKey(1)

    def clean(self, img):
        cv2.destroyAllWindows()

    def denoiseRaw(self, img):
        return cv2.medianBlur(img, 3)

    def drawContourInfo(self, img, cinfo, coordinates = None):
        cv2.rectangle(img, cinfo["box"][0], cinfo["box"][1], (255,0,255))
        center = tuple(int(round(dim)) for dim in cinfo["centroid"])
        cv2.circle(img, center, 2, (255,0,255))

        if coordinates is not None:
            color = (255,0,255) if coordinates[2] else (0,0,255)
            text = "{:.2f}, {:.2f}".format(coordinates[0], coordinates[1])
            cv2.putText(img, text, cinfo["box"][0], cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def contourInfo(self, contour):
        contourVec = np.reshape(contour, (-1, 2))
        contourDict = {}

        try:
            coordsX = contourVec[:, 0]
        except IndexError as e:
            print(contourVec)
            print(np.shape(contourVec))
            raise e
        coordsY = contourVec[:, 1]
        box = ((np.min(coordsX), np.min(coordsY)),
               (np.max(coordsX), np.max(coordsY)))
        width = box[1][0] - box[0][0]
        height = box[1][1] - box[0][1]
        contourDict["centroid"] = (np.mean(coordsX), np.mean(coordsY))
        contourDict["box"] = box
        contourDict["dimensions"] = (width, height)
        contourDict["area"] = cv2.contourArea(contour)

        arcLength = cv2.arcLength(contour, False)
        contourDict["perimeter"] = arcLength
        if arcLength > 0:
            contourDict["circularity"] = 4 * math.pi * contourDict["area"] / (math.pow(contourDict["perimeter"], 2))
        else:
            contourDict["circularity"] = 1

        try:
            contourDict["aspect"] = float(width) / float(height)
        except ZeroDivisionError as e:
            contourDict["aspect"] = float('inf')

        contourDict["raw"] = contour

        return contourDict

    def realCoordinates(self, cinfo, props):
        realAspect = props["dimensions"][0] / props["dimensions"][1]
        angles = np.multiply(np.divide(cinfo["dimensions"], self.res), self.fov)
        zValues = np.divide(props["dimensions"], np.tan(angles))
        z = np.mean(zValues)

        reliable = True
        halfRes = np.multiply(self.res, 0.5)
        bottomY = cinfo["box"][1][1]

        if (self.res[1] - bottomY / self.res[1]) <= 0.01 or abs(realAspect - cinfo["aspect"]) >= 0.2:
            reliable = False
            vertAngle = self.tilt - ((bottomY - halfRes[1]) / halfRes[1] * (self.fov[1] / 2))
            if vertAngle >= (math.pi / 2):
                return None
            z = self.camHeight * math.tan(vertAngle)

        xAngle = (cinfo["centroid"][0] - halfRes[0]) / halfRes[0] * (self.fov[0] / 2)
        x = z * math.tan(xAngle)

        return x, z, reliable


    def ballThreshold(self, hsv):
        ballLower = np.array([0, 100, 100])
        ballUpper = np.array([15, 255, 255])
        ballMask = cv2.inRange(hsv, ballLower, ballUpper)
        return self.open(ballMask, 3)

    def grassThreshold(self, hsv):
        ballLower = np.array([50, 0, 0])
        ballUpper = np.array([100, 255, 255])
        ballMask = cv2.inRange(hsv, ballLower, ballUpper)
        return self.open(ballMask, 9)

    def open(self, img, kernelSize):
        return cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones(kernelSize))

    def wallThreshold(self, hsv):
        wallLower = np.array([0, 0, 190])
        wallUpper = np.array([255, 50, 255])
        wallMask = cv2.inRange(hsv, wallLower, wallUpper)
        return self.open(wallMask, 9)

    def obstacleThreshold(self, hsv):
        obstacleLower = np.array([0, 200, 0])
        obstacleUpper = np.array([255, 255, 50])
        obstacleMask = cv2.inRange(hsv, obstacleLower, obstacleUpper)
        return self.open(obstacleMask, 9)

    def ballBlobs(self, thresholded):
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
        return  ballDetector.detect(thresholded)

    def findContours(self, thresholded):
        something, contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours if len(contours) > 0 else None