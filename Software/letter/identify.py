import cv2
import numpy as np
import operator
import os
import sys

RESIZED_IMAGE_WIDTH = 12
RESIZED_IMAGE_HEIGHT = 12
BLOCKSIZE = 131
MIN_SIZE = 5000
C = 5
PATH = sys.argv[1]

""" ERRORS
0 - no contour found
1 - letter not recognizable (equal black and white pixels in ROI
2 - letter too small
3 - letter not dark enough/background is white
4 - ratio too big/small
5 - letter touching edge of photo
"""

class ContourWithData():
    contour = None
    boundingRect = None
    rectX = 0
    rectY = 0
    rectWidth = 0
    rectHeight = 0

    def calculateCoords(self):
        [intX, intY, intWidth, intHeight] = self.boundingRect
        self.rectX = intX
        self.rectY = intY
        self.rectHeight = intHeight
        self.rectWidth = intWidth

    def touchingEdge(self):
        if self.rectX == 0 or self.rectY == 0:
            return True
        elif self.rectX + self.rectWidth == 720 or self.rectY + self.rectHeight == 480:
            return True
        else:
            return False


def solveLetter(top, bottom):
    whitePixelsTop = 0
    whitePixelsBottom = 0

    for pixel in top:
        if pixel == 255: whitePixelsTop += 1
        elif pixel == 0: whitePixelsTop -= 1
    for pixel in bottom:
        if pixel == 255: whitePixelsBottom += 1
        elif pixel == 0: whitePixelsBottom -= 1

    if whitePixelsTop > 0: whitePixelsTop = 1
    elif whitePixelsTop < 0: whitePixelsTop = 0
    else: whitePixelsTop = -10
    if whitePixelsBottom > 0: whitePixelsBottom = 1
    elif whitePixelsBottom < 0: whitePixelsBottom = 0
    else: whitePixelsBottom = -10

    sum = whitePixelsTop + whitePixelsBottom
    if sum == 2: return 'S' #S
    elif sum == 0: return 'H' #H
    elif whitePixelsTop == 0 and whitePixelsBottom == 1: return 'U' #U
    else: return -1
    

def findContour(contours):
    maxContour = contours[0]
    for contour in contours:
        if cv2.contourArea(contour) > cv2.contourArea(maxContour):
            maxContour = contour
    return maxContour


def genContourData(maxContour):
    data = ContourWithData()
    data.contour = maxContour
    data.boundingRect = cv2.boundingRect(data.contour)
    data.calculateCoords()
    return data


def genTopAndBottom(thres, contourData):
    imgROI = thres[contourData.rectY : contourData.rectY+contourData.rectHeight, contourData.rectX : contourData.rectX+contourData.rectWidth]
    imgROIResized = cv2.resize(imgROI, (RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT))
    imgROIResized = cv2.adaptiveThreshold(imgROIResized, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, BLOCKSIZE, C)
    top = imgROIResized[0: 1, 4: 7][0]
    bottom = imgROIResized[11: 12, 4: 7][0]
    return top, bottom

def main2():
    imgOriginal = cv2.imread(PATH)
    imgOriginal = cv2.resize(imgOriginal, (720, 480))
    imgGray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
    imgBlurred = cv2.GaussianBlur(imgGray, (11, 11), 0)
    imgThresh = cv2.adaptiveThreshold(imgBlurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, BLOCKSIZE, C)
    imgThreshCopy = imgThresh.copy()
    contours, _ = cv2.findContours(imgThreshCopy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        maxContour = findContour(contours)
    else:
        print (0)
        return

    contourData = genContourData(maxContour)
    topROI, bottomROI = genTopAndBottom(imgThresh, contourData)
    result = solveLetter(topROI, bottomROI)
    ratio = contourData.rectHeight/contourData.rectWidth

    if result == -1 or cv2.contourArea(maxContour) < MIN_SIZE or ratio < 0.9 or ratio > 1.9 or contourData.touchingEdge():
        print (0)
    else:
        print(result)
    return


def main():
    main2()


if __name__ == "__main__":
    main()
