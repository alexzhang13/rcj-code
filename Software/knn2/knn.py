import cv2
import numpy
import operator
import os
import sys

RESIZED_IMAGE_WIDTH = 32
RESIZED_IMAGE_HEIGHT = 32
BLOCKSIZE = 131
C = 5
DIST = 10**8
PATH = "newdata/H/0.jpg"

class ContourWithData():
    npaContour = None
    boundingRect = None
    intRectX = 0
    intRectY = 0
    intRectWidth = 0
    intRectHeight = 0

    def calculateCoords(self):
        [intX, intY, intWidth, intHeight] = self.boundingRect
        self.intRectX = intX
        self.intRectY = intY
        self.intRectHeight = intHeight
        self.intRectWidth = intWidth

def main():
    allContoursWithData = []

    npaClassifications = numpy.loadtxt("classifications.txt", numpy.float32)
    npaFlattenedImages = numpy.loadtxt("flattened_images.txt", numpy.float32)

    npaClassifications = npaClassifications.reshape((npaClassifications.size,1))
    kNearest = cv2.ml.KNearest_create()
    kNearest.train(npaFlattenedImages, cv2.ml.ROW_SAMPLE, npaClassifications)

    imgTestingNumbers = cv2.imread(PATH)
    imgGray = cv2.cvtColor(imgTestingNumbers, cv2.COLOR_BGR2GRAY)
    imgBlurred = cv2.GaussianBlur(imgGray, (11, 11), 0)
    imgThresh = cv2.adaptiveThreshold(imgBlurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, BLOCKSIZE, C)

    imgThreshCopy = imgThresh.copy()
    _, npaContours, _ = cv2.findContours(imgThreshCopy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(npaContours) > 0:
        maxContour = npaContours[0]
        for npaContour in npaContours:
            if cv2.contourArea(npaContour) > cv2.contourArea(maxContour):
                maxContour = npaContour
    else:
        print ("no contours found")
        return

    cWithData = ContourWithData()
    cWithData.npaContour = maxContour
    cWithData.boundingRect = cv2.boundingRect(cWithData.npaContour)
    cWithData.calculateCoords()
    allContoursWithData.append(cWithData)

    imgROI = imgThresh[cWithData.intRectY : cWithData.intRectY+cWithData.intRectHeight,
                       cWithData.intRectX : cWithData.intRectX+cWithData.intRectWidth]
    imgROIResized = cv2.resize(imgROI, (RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT))
    npaROIResized = imgROIResized.reshape((1, RESIZED_IMAGE_HEIGHT*RESIZED_IMAGE_WIDTH))
    npaROIResized = numpy.float32(npaROIResized)

    _, npaResults, _, dists = kNearest.findNearest(npaROIResized, k=1)

    character = str(chr(int(npaResults[0][0])))

    #print (character)
    print (dists[0][0])
    if dists[0][0] < DIST:
        sys.exit(character)
    else:
        sys.exit(0)
    return

if __name__ == "__main__":
    main()
