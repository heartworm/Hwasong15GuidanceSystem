import cv2
import numpy as np
import math

#CamelCase is used for consistency with openCV styling

class ImageAnalyser:
    def __init__(self):
        # self.fov = np.array((math.radians(80), math.radians(50)))
        self.fov = np.array((math.radians(62.2) , math.radians(48.8)))
        self.tilt = math.radians(73.5)
        # self.camHeight = 0.21
        self.camHeight = 0.15
        self.res = np.array((-1,-1))
        self.ballProps = {
            "dimensions": (0.0427, 0.0427)
        }
        self.obstacleProps = {
            "dimensions": (0.18, 0.225)
        }

        self.goalProps = {
            "dimensions": (0.7, 0.225)
        }

        self.ballPos = None
        self.goalPos = None
        self.wallPoses = []
        self.obstaclePoses = []

    def analyse(self, img):
        self.res = np.array(np.shape(img)[1::-1], dtype='float')
        self.area = self.res[0] * self.res[1]
        denoised = self.denoiseRaw(img)

        hsv = cv2.cvtColor(denoised, cv2.COLOR_BGR2HSV)
        areaGetter = lambda info: info["area"]

        ballMask = self.ballThreshold(hsv)
        ballContours = self.findContours(np.array(ballMask))
        ballMask = cv2.cvtColor(ballMask, cv2.COLOR_GRAY2BGR)
        if len(ballContours) > 0:
            ballInfos = sorted([self.contourInfo(contour) for contour in ballContours], key=areaGetter, reverse=True)
            ballInfo = ballInfos[0]
            self.ballPos = self.realCoordinates(ballInfo, self.ballProps)
            polar, cartesian, reliable = self.ballPos
            if ballInfo["area"] >= self.area * 0.0001 and cartesian[0] is not None:
                self.drawContourInfo(denoised, ballInfo, color=(255,0,255))
                text = "{:.2f}, {:.2f}, {}".format(cartesian[0], cartesian[1], reliable)
                cv2.putText(ballMask, text, (10, int(self.res[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255))
                text = "{:.2f}, {:.2f}".format(math.degrees(polar[0]), polar[1])
                cv2.putText(ballMask, text, (10, int(self.res[1] - 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255))
            else:
                self.ballPos = None
        else:
            self.ballPos = None

        cv2.imshow('ball', ballMask)

        obstacleMask = self.obstacleThreshold(hsv)
        obstacleContours = self.findContours(np.array(obstacleMask))
        obstacleMask = cv2.cvtColor(obstacleMask, cv2.COLOR_GRAY2BGR)
        obstacleInfos = [self.contourInfo(contour) for contour in obstacleContours]
        obstacleInfos = [info for info in obstacleInfos if info["area"] >= self.area * 0.02]
        obstacleInfos = sorted(obstacleInfos, key=areaGetter, reverse=True)
        self.obstaclePoses = []

        for obstacleInfo in obstacleInfos:
            pos = self.realCoordinates(obstacleInfo, self.obstacleProps)
            polar, cartesian, reliable = pos

            infoText = None
            if polar[1] is not None:
                self.obstaclePoses.append(pos)
                infoText = "{:.2f}, {:.2f}, {}".format(math.degrees(polar[0]), polar[1], reliable)
                text = "{:.2f}, {:.2f}, {}".format(cartesian[0], cartesian[1], reliable)
                cv2.putText(obstacleMask, text, (10, int(self.res[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255))
                text = "{:.2f}, {:.2f}".format(math.degrees(polar[0]), polar[1])
                cv2.putText(obstacleMask, text, (10, int(self.res[1] - 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255))

            self.drawContourInfo(denoised, obstacleInfo, color=(0,0,255), text=infoText)
        cv2.imshow('obstacle', obstacleMask)

        grassMask = self.grassThreshold(hsv)

        wallMask = self.wallThreshold(hsv)


        wallBase = self.wallBase(grassMask, wallMask, distanceThreshold=5)

        wallDisp = cv2.cvtColor(wallMask, cv2.COLOR_GRAY2BGR)
        wallDisp[:,:,1] = np.maximum(wallDisp[:,:,1], grassMask)


        if len(wallBase) > 0:
            self.wallPoses = [self.pointToCoordinates(point) for point in wallBase]
            self.wallPoses = [(polar, cartesian, False) for polar, cartesian in self.wallPoses]

            closestInd = np.argmax(wallBase[:,1])
            closestPoint = tuple(wallBase[closestInd, :])

            for x, y in wallBase:
                cv2.circle(wallDisp, (x, y), 2, (255,0,0))

            cv2.circle(wallDisp, closestPoint, 4, (255,0,255), 3)
            polar, cartesian = self.pointToCoordinates(closestPoint)
            if polar[1] is not None:
                cv2.putText(wallDisp, "{:2f}, {:2f}".format(math.degrees(polar[0]), polar[1]),
                        (10, int(self.res[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255))
        else:
            self.wallPoses = []

        wallBaseMask = np.zeros(np.shape(wallMask), dtype='uint8')
        for x, y in wallBase:
            wallBaseMask[y, x] = 255

        lines = cv2.HoughLines(wallBaseMask, 1, math.radians(5), 30)
        if lines is not None:
            lines = np.squeeze(lines, axis=1)
            for rho, theta in lines:
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a*rho
                y0 = b*rho
                p0 = (int(round(x0 + 1000 * (-b))), int(round(y0 + 1000 * a)))
                p1 = (int(round(x0 - 1000 * (-b))), int(round(y0 - 1000 * a)))
                cv2.line(wallDisp, p0, p1, (0,0,255), 1)

        cv2.imshow('wallgrass', wallDisp)

        goalMask = self.goalThreshold(hsv)

        goalContours = self.findContours(np.array(goalMask))
        goalMask = cv2.cvtColor(goalMask, cv2.COLOR_GRAY2BGR)
        goalInfos = [self.contourInfo(contour) for contour in goalContours]
        goalInfos = [info for info in goalInfos if info["area"] >= self.area * 0.01]
        goalInfos = sorted(goalInfos, key=areaGetter, reverse=True)
        if len(goalInfos) > 0:
            goalInfo = goalInfos[0]
            self.goalPos = self.realCoordinates(goalInfo, self.goalProps)
            polar, cartesian, reliable = self.goalPos
            infoText = None
            if polar[1] is not None:
                text = "{:.2f}, {:.2f}, {}".format(cartesian[0], cartesian[1], reliable)
                cv2.putText(goalMask, text, (10, int(self.res[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255))
                text = "{:.2f}, {:.2f}".format(math.degrees(polar[0]), polar[1])
                cv2.putText(goalMask, text, (10, int(self.res[1] - 30)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255))
                infoText = "{:.2f}, {:.2f}, {}".format(math.degrees(polar[0]), polar[1], reliable)
            else:
                self.goalPos = None
            self.drawContourInfo(denoised, goalInfo, color=(0,255,0), text=infoText)
        else:
            self.goalPos = None

        cv2.imshow('goals', goalMask)

        cv2.imshow('denoised', denoised)
        cv2.waitKey(1)

    def clean(self, img):
        cv2.destroyAllWindows()

    def denoiseRaw(self, img):
        return cv2.medianBlur(img, 3)

    def drawContourInfo(self, img, cinfo, text = None, color = None, thickness = None):
        color = (255,0,255) if color is None else color
        thickness = 2 if thickness is None else color
        cv2.rectangle(img, cinfo["box"][0], cinfo["box"][1], color, thickness)
        center = tuple(int(round(dim)) for dim in cinfo["centroid"])
        cv2.circle(img, center, 2, color)

        if text is not None:
            cv2.putText(img, text, cinfo["box"][0], cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def wallBase(self, grassMask, wallMask, distanceThreshold):
        basePoints = {}
        grassSobel = cv2.Sobel(grassMask, -1, 0, 1)
        edgeRows, edgeCols  = np.nonzero(grassSobel)
        for n in range(len(edgeRows)):
            row = edgeRows[n]
            col = edgeCols[n]
            if np.max(wallMask[max(0, row-distanceThreshold):row, col]) == 255:
                basePoints[col] = row

        maxY = int(self.res[1] - 1)
        wallBottomCols = np.nonzero(wallMask[maxY, :])[0]
        for wallBottomCol in wallBottomCols:
            if wallBottomCol not in basePoints:
                basePoints[wallBottomCol] = maxY

        return np.array(list(basePoints.items()))

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
        bottomY = cinfo["box"][1][1]

        polar, cartesian = self.pointToCoordinates((cinfo["centroid"][0], bottomY))
        xAngle, range = polar
        x, z = cartesian

        reliable = not ((self.res[1] - bottomY / self.res[1]) <= 0.01 or abs(realAspect - cinfo["aspect"]) >= 0.2)
        if reliable:
            angles = np.multiply(np.divide(cinfo["dimensions"], self.res), self.fov)
            zValues = np.divide(props["dimensions"], np.tan(angles))
            z = np.mean(zValues)
            x = z * math.tan(xAngle)
            range = math.sqrt(x * x + z * z)

        return (xAngle, range), (x, z), reliable

    def pointToCoordinates(self, point):
        halfRes = np.multiply(self.res, 0.5)
        xin, yin = point
        vertAngle = self.tilt - ((yin - halfRes[1]) / halfRes[1] * (self.fov[1] / 2))
        xAngle = (xin - halfRes[0]) / halfRes[0] * (self.fov[0] / 2)

        if vertAngle >= (math.pi / 2):
            x = None
            z = None
            range = None
        else:
            z = self.camHeight * math.tan(vertAngle)
            x = z * math.tan(xAngle)
            range = math.sqrt(x * x + z * z)

        polar = (xAngle, range)
        cartesian = (x, z)
        return polar, cartesian


    def ballThreshold(self, hsv):
        ballLower = np.array([0, 100, 100])
        ballUpper = np.array([15, 255, 255])
        ballMask = cv2.inRange(hsv, ballLower, ballUpper)
        return self.open(ballMask, 3)

    def grassThreshold(self, hsv):
        ballLower = np.array([40, 30, 25])
        ballUpper = np.array([60, 255, 255])
        ballMask = cv2.inRange(hsv, ballLower, ballUpper)
        return self.open(ballMask, 9)

    def open(self, img, kernelSize):
        return cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones(kernelSize))

    def wallThreshold(self, hsv):
        wallLower = np.array([0, 0, 100])
        wallUpper = np.array([255, 50, 255])
        wallMask = cv2.inRange(hsv, wallLower, wallUpper)
        return self.open(wallMask, 9)

    def obstacleThreshold(self, hsv):
        # obstacleLower = np.array([0, 200, 0])
        obstacleLower = np.array([0, 0, 0])
        obstacleUpper = np.array([30, 255, 25])
        obstacleMask = cv2.inRange(hsv, obstacleLower, obstacleUpper)
        return self.open(obstacleMask, 9)

    def goalThreshold(self, hsv):
        blueLower = np.array([95, 128, 63])
        blueUpper = np.array([110, 255, 255])
        yellowLower = np.array([22, 110, 110])
        yellowUpper = np.array([33, 255, 255])
        blueMask = cv2.inRange(hsv, blueLower, blueUpper)
        yellowMask = cv2.inRange(hsv, yellowLower, yellowUpper)
        goalMask = np.maximum(blueMask, yellowMask)
        return self.open(goalMask, 9)

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
        return contours if len(contours) > 0 else []