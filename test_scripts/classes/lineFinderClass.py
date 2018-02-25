import numpy as np
from lineCollectorClass import LineCollector
import cv2
from constants import DISPLAY_IMAGE, GAUSS_KERNEL, \
    CANNY_LOW_THRESH, CANNY_HIGH_THRESH, REGION_OF_INTEREST, \
    HOUGH_RHO, HOUGH_THETA, HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP, \
    BLUE_HUE, BLUE_HUE_THRESH, BLUE_SAT_MIN, BLUE_SAT_MAX, BLUE_VAL_MIN, BLUE_VAL_MAX

class LineFinder:

    def __init__(self):
        self.lineCollector = LineCollector()

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
        can_raw = self.canny(filtered, CANNY_LOW_THRESH, CANNY_HIGH_THRESH)
        lines = self.hough_lines(can_raw, HOUGH_RHO, HOUGH_THETA, \
            HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP)

        single_line = self.lineCollector.collect(line)

        if DISPLAY_IMAGE:
            if lines is None or len(lines) == 0 or lines[0] == []:
                lines = [[[0, 0, 0, 0]]]
            lines = self.removeThatStupidDimension(lines)
            raw_hough = self.newLinesImage(lines, self.w, self.h)
            can = cv2.cvtColor(can_raw, cv2.COLOR_GRAY2BGR)
            singleLineImage = newLinesImage(single_line, self.w, self.h)
            output = self.combineImages(\
                (image, filtered, can, region, raw_hough, singleLineImage))
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

    def canny(self, img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        return cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

    def newLinesImage(self, lines, w, h):
        image = np.zeros((h, w, 3), dtype=np.uint8)
        self.draw_lines(image, lines, thickness=5)
        return image

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            x1,y1,x2,y2 = line
            cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)

    def showImage(image):
        cv2.imshow('image', image)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            pass

    def combineImages(images):
        a = np.concatenate((images[0], images[1], images[2]), axis=1)
        b = np.concatenate((images[3], images[4], images[5]), axis=1)
        return np.concatenate((a, b), axis=0)
