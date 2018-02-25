#!/usr/bin/python
import numpy as np
from lineHistoryClass import LineHistoryClass
import cv2
from constants import DISPLAY_IMAGE, GAUSS_KERNEL, \
    CANNY_LOW_THRESH, CANNY_HIGH_THRESH, REGION_OF_INTEREST, \
    HOUGH_RHO, HOUGH_THETA, HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP, \
    BLUE_HUE, BLUE_HUE_THRESH, BLUE_SAT_MIN, BLUE_SAT_MAX, BLUE_VAL_MIN, BLUE_VAL_MAX, \
    SLOPE_MIN_THRESH, HISTORY_SIZE

class LineFinder:

    def __init__(self):
        self.history = LineHistoryClass(HISTORY_SIZE)

    def findIn(self, image):
        self.h, self.w, _ = image.shape
        try:
            return self.lineCoordsFromImage(image)[0]
        except IndexError:
            return []

    def lineCoordsFromImage(self, image):
        region_of_interest_pixels = self.convertToPixelRegion(REGION_OF_INTEREST, self.w, self.h)
        region = self.region_of_interest(image, [region_of_interest_pixels])
        filtered = self.colorFilter(cv2.cvtColor(region,cv2.COLOR_BGR2HSV), \
            BLUE_HUE, BLUE_HUE_THRESH, BLUE_SAT_MIN, BLUE_SAT_MAX, BLUE_VAL_MIN, BLUE_VAL_MAX)
        blur = self.gaussian_blur(filtered, GAUSS_KERNEL)
        can_raw = self.canny(blur, CANNY_LOW_THRESH, CANNY_HIGH_THRESH)
        lines = self.hough_lines(can_raw, HOUGH_RHO, HOUGH_THETA, \
            HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP)

        single_line = self.processLines(lines)

        if DISPLAY_IMAGE:
            if lines is None or len(lines) == 0 or lines[0] == []:
                lines = [[[0, 0, 0, 0]]]
            lines = self.removeThatStupidDimension(lines)
            raw_hough = self.newLinesImage(lines, self.w, self.h)
            can = cv2.cvtColor(can_raw, cv2.COLOR_GRAY2BGR)
            output = self.combineImages(\
                (image, filtered, can, region, raw_hough, newLinesImage(single_line, self.w, self.h)))
            self.showImage(output)

        return single_line

    def convertToPixelRegion(self, region, width, height):
        pixels = np.empty((region.shape[0], 2), np.int32)
        for i in range(region.shape[0]):
            x = region[i][0]
            y = region[i][1]
            pixels[i] = [int(x * width), int(y * height)]
        return pixels

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def colorFilter(self, image, hue, hue_thresh, sat_min, sat_max, val_min, val_max):
        lower = np.array([hue - hue_thresh, sat_min, val_min])
        upper = np.array([hue + hue_thresh, sat_max, val_max])
        mask = cv2.inRange(image, lower, upper)
        return cv2.bitwise_and(image, image, mask=mask)

    def gaussian_blur(self, img, kernel_size):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def canny(self, img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    def grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        return cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    def processLines(self, lines):
        if lines is None:
            return []
        else:
            lines = self.removeThatStupidDimension(lines)
            lines = self.filterBySlope(lines)
            lines = self.ensureLinesGoTopToBottom(lines)
            avg_line = self.averageLines(lines, self.history.getLastLine())
            self.history.addToHistory(avg_line)
            moving_left_avg = self.history.getAverage()
            return [moving_left_avg]

    def ensureLinesGoTopToBottom(self, lines):
        size = len(lines)
        for i in range(size):
            [x1, y1, x2, y2] = lines[i]
            if y1 > y2:
                lines[i] = [x2, y2, x1, y1]
        return lines

    def removeThatStupidDimension(self, lines):
        output = []
        for line in lines:
            output.append(line[0])
        return output

    def filterBySlope(self, lines):
        output = []
        for line in lines:
            m = self.slope(line)
            if abs(m) > SLOPE_MIN_THRESH:
                output.append(line)
        return output

    def averageLines(self, lines, default):
        if len(lines) == 0:
            return default
        avg_line = [0] * 4
        for line in lines:
            for i in range(4):
                avg_line[i] += line[i]
        for i in range(4):
            avg_line[i] /= len(lines)
        return avg_line

    def slope(self, line):
        x1, y1, x2, y2 = line
        if x1 == x2:
            return float('inf')
        return (float(y1) - float(y2)) / (float(x1) - float(x2))

    def newLinesImage(self, lines, w, h):
        image = np.zeros((h, w, 3), dtype=np.uint8)
        self.draw_lines(image, lines, thickness=5)
        return image

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            x1,y1,x2,y2 = line
            cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)

    def showImage(image):
        if DISPLAY_IMAGE:
            cv2.imshow('image', image)
            if cv2.waitKey(20) & 0xFF == ord('q'):
    	           pass

    def combineImages(images):
        a = np.concatenate((images[0], images[1], images[2]), axis=1)
        b = np.concatenate((images[3], images[4], images[5]), axis=1)
        return np.concatenate((a, b), axis=0)
