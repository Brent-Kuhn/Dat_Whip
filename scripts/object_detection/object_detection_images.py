#!/usr/bin/python

def areaOfQuadrilateral(quad):
    # Using the shoelace formula for a quadrilateral
    x1 = quad[0][0]
    y1 = quad[0][1]
    x2 = quad[1][0]
    y2 = quad[1][1]
    x3 = quad[2][0]
    y3 = quad[2][1]
    x4 = quad[3][0]
    y4 = quad[3][1]
    return .5 * abs(x1*y2 + x2*y3 + x3*y4 + x4*y1 - x2*y1 - x3*y2 - x4*y3 - x1*y4)

import cv2
import numpy as np


IMAGE_WIDTH = 672
IMAGE_HEIGHT = 376

ORANGE_MIN = [0, 200, 40]
ORANGE_MAX = [13, 255, 255]
ORANGE_REGION = [[0, .4], [0, .2], [1, .2], [1, .4]]
ORANGE_AREA = areaOfQuadrilateral(ORANGE_REGION)

BLUE_MIN = [110, 150, 40]
BLUE_MAX = [115, 255, 255]
BLUE_REGION = ORANGE_REGION
BLUE_AREA = areaOfQuadrilateral(BLUE_REGION)

COLOR_THRESHOLD = 0.0075

class ObjectDetector():

    def findMainObject(self, zed):
        leftImage = zed[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH]
        hsv = cv2.cvtColor(leftImage, cv2.COLOR_BGR2HSV)
        self.debugImage = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), np.uint8)
        orange = self.calcPercentOfColorInRegion(hsv, ORANGE_MIN, ORANGE_MAX, ORANGE_REGION)
        orange *= .5
        blue = self.calcPercentOfColorInRegion(hsv, BLUE_MIN, BLUE_MAX, BLUE_REGION)
        obj = max(orange, blue)
        if obj < COLOR_THRESHOLD:
            return ''
        return 'cone' if obj == blue else 'cube'

    def calcPercentOfColorInRegion(self, hsv, colorMin, colorMax, region):
        roi = ObjectDetector.regionOfInterest(hsv, np.array(region))
        mask = ObjectDetector.filterColor(roi, colorMin, colorMax)
        numWhitePixels = np.sum(mask == 255)
        totalPixels = float(hsv.shape[0] * hsv.shape[1]) * ORANGE_AREA
        # showImages((
        #     cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR),
        #     cv2.cvtColor(roi, cv2.COLOR_HSV2BGR),
        #     cv2.bitwise_and(cv2.cvtColor(roi, cv2.COLOR_HSV2BGR), cv2.cvtColor(roi, cv2.COLOR_HSV2BGR), mask=mask)))\
        output = cv2.bitwise_and(cv2.cvtColor(roi, cv2.COLOR_HSV2BGR), cv2.cvtColor(roi, cv2.COLOR_HSV2BGR), mask=mask)
        self.debugImage = cv2.bitwise_or(self.debugImage, output)
        return numWhitePixels / totalPixels

    @staticmethod
    def regionOfInterest(img, vertices):
        vertices = ObjectDetector.convertToPixelRegion(vertices, img.shape[1], img.shape[0])
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        cv2.fillPoly(mask, [vertices], ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    @staticmethod
    def convertToPixelRegion(region, width, height):
        pixels = np.empty((region.shape[0], 2), np.int32)
        for i in range(region.shape[0]):
            x = region[i][0]
            y = region[i][1]
            pixels[i] = [int(x * width), int(y * height)]
        return pixels

    @staticmethod
    def filterColor(hsv, colorMin, colorMax):
        return cv2.inRange(hsv, np.array(colorMin), np.array(colorMax))

    @staticmethod
    def showBwImage(image):
        ObjectDetector.showImage(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))

    @staticmethod
    def showHsvImage(image):
        ObjectDetector.showImage(cv2.cvtColor(image, cv2.COLOR_HSV2BGR))

    @staticmethod
    def showImage(image):
        cv2.imshow('zed', image)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            pass

    @staticmethod
    def showImages(images):
        combined = np.concatenate(images, axis=1)
        ObjectDetector.showImage(combined)
