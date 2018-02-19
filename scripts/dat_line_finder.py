#!/usr/bin/python
import cv2
import math
import rospy as rp
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

pub = None

### Custom constants to tweak for ultimate perfection ###
SHOW_ALL_STEPS = False
GAUSS_KERNEL = 5

CANNY_LOW_THRESH = 50
CANNY_HIGH_THRESH = 75

REGION_OF_INTEREST = np.array(\
    [[0.45, 0.6], [0.55, 0.6], [1.3, 1], [-.3, 1]], np.float32)

MULT = 1
HOUGH_RHO = 1
HOUGH_THETA = np.pi / 180
HOUGH_THRESH = 20
HOUGH_MIN_LEN = 50 * MULT
HOUGH_MAX_GAP = 30 * MULT

ORIGINAL_IMAGE_WEIGHT = 0.5
IMAGE_SCALE = 0.7

MOVING_AVG_SIZE = 3

SLOPE_MIN = .1


def main():
    global pub
    subscribeToLeft()
    rp.init_node('follow_coords', anonymous=True)
    pub = rp.Publisher('xy', String, queue_size=10)

def subscribeToLeft():
    rp.init_node('camera', anonymous=True)
    rp.Subscriber('zedLeft', Image, zedLeftCallback)
	rp.spin()

def zedLeftCallback(data):
    image = getCVImageFromData(data)
    line = lineCoordsFromImage(image)
    x, y = getXYFromLine(line)
    publishXY(x, y)

def getCVImageFromData(data):
    return bridge.imgmsg_to_cv2(data, encoding='passthrough')

def lineCoordsFromImage(image, name):
    h, w, _ = image.shape

    blur = gaussian_blur(image, GAUSS_KERNEL)

    filtered = colorFilter(blur, 200, 40, 0, 150, 0, 200)

    can_raw = canny(filtered, CANNY_LOW_THRESH, CANNY_HIGH_THRESH)
    can = cv2.cvtColor(can_raw, cv2.COLOR_GRAY2BGR)

    region_of_interest_pixels = convertToPixelRegion(REGION_OF_INTEREST, w, h)
    region = region_of_interest(can, [region_of_interest_pixels])
    region_bw = grayscale(region)

    lines = hough_lines(region_bw, HOUGH_RHO, HOUGH_THETA, \
        HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP)
    return processLines(lines)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def colorFilter(image, hue, hue_thresh, sat_min, sat_max, val_min, val_max):
    lower = np.array([hue - hue_thresh, sat_min, val_min])
    upper = np.array([hue + hue_thresh, sat_max, val_max])
    mask = cv2.inRange(image, lower, upper)
    blur = gaussian_blur(mask, GAUSS_KERNEL)
    return cv2.bitwise_and(image, image, mask=blur)

def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def convertToPixelRegion(region, width, height):
    pixels = np.empty((region.shape[0], 2), np.int32)
    for i in range(region.shape[0]):
        x = region[i][0]
        y = region[i][1]
        pixels[i] = [int(x * width), int(y * height)]
    return pixels

def region_of_interest(img, vertices):
    """
    Applies an image mask.

    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    """
    return cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

def processLines(lines):
    global last_left, last_right, last_left_lines, last_right_lines, last_index

    if lines is None:
        moving_left_avg = last_left
        moving_right_avg = last_right
    else:
        lines = removeThatStupidDimension(lines)
        lines = filterBySlope(lines)
        lines = ensureLinesGoTopToBottom(lines)

        left_lines = lines #, right_lines = sortLeftAndRight(lines)

        left_avg = averageLines(left_lines, last_left)
        # right_avg = averageLines(right_lines, last_right)

        if True:
            addToHistory(last_left_lines, left_avg, last_index)
            # addToHistory(last_right_lines, right_avg, last_index)
            last_index += 1

            moving_left_avg = averageHistory(last_left_lines)
            # moving_right_avg = averageHistory(last_right_lines)
        # else:
        #     moving_left_avg = left_avg
        #     # moving_right_avg = right_avg

        last_left = left_avg
        # last_right = right_avg

    l = extrapolateToBottom(moving_left_avg)
    # r = extrapolateToBottom(moving_right_avg)
    # l = extrapolateToMid(l)
    # r = extrapolateToMid(r)

    # return [l, r]
    return [l]

def removeThatStupidDimension(lines):
    output = []
    for line in lines:
        output.append(line[0])
    return output

def filterBySlope(lines):
    output = []
    for line in lines:
        m = slope(line)
        if abs(m) > SLOPE_MIN:
            output.append(line)
    return output

def averageHistory(last_lines):
    moving_avg_line = [0] * 4
    for i in range(4):
        for line in last_lines:
            moving_avg_line[i] += line[i]
        moving_avg_line[i] /= MOVING_AVG_SIZE
    return moving_avg_line

def addToHistory(last_lines, avg_line, last_index):
    if np.sum(last_lines) == 0:
        fillHistoryWith(last_lines, avg_line)
    for i in range(4):
        last_lines[last_index % MOVING_AVG_SIZE][i] = avg_line[i]

def fillHistoryWith(last_lines, line):
    for i in range(MOVING_AVG_SIZE):
        for j in range(4):
            last_lines[i][j] = line[j]

def sortLeftAndRight(lines):
    left_lines = []
    right_lines = []
    for line in lines:
        if slope(line) < 0:
            left_lines.append(line)
        else:
            right_lines.append(line)
    return left_lines, right_lines

def averageLines(lines, default):
    if len(lines) == 0:
        return default
    avg_line = [0] * 4
    for line in lines:
        for i in range(4):
            avg_line[i] += line[i]
    for i in range(4):
        avg_line[i] /= len(lines)
    return avg_line

def extrapolateToBottom(line):
    global image_height
    x1, y1, x2, y2 = line
    x1 = float(x1)
    y1 = float(y1)
    x2 = float(x2)
    y2 = float(y2)
    m = slope(line)
    if m == 0 or math.isnan(m) or math.isinf(m):
        return [0, 0, 0, 0]
    newY = image_height
    newX = (newY - y1 + m * x1) / m
    if y1 > y2:
        y1 = newY
        x1 = int(newX)
    else:
        y2 = newY
        x2 = int(newX)
    return [x1, y1, x2, y2]

def extrapolateToMid(line):
    global image_height
    x1, y1, x2, y2 = line
    x1 = float(x1)
    y1 = float(y1)
    x2 = float(x2)
    y2 = float(y2)
    m = slope(line)
    if m == 0:
        return [0, 0, 0, 0]
    newY = int(image_height * .7)
    newX = (newY - y1 + m * x1) / m
    from math import isnan
    if isnan(newX):
        return [x1, y1, x2, y2]
    if y1 > y2:
        y2 = newY
        x2 = int(newX)
    else:
        y1 = newY
        x1 = int(newX)
    return [x1, y1, x2, y2]

def slope(line):
    x1, y1, x2, y2 = line
    if x1 == x2:
        return float('inf')
    return (float(y1) - float(y2)) / (float(x1) - float(x2))






def getXYFromLine(line, width, height):
    [topX, topY, _, _] = line
    x = topX - int(width / 2)
    y = height - topY
    return x, y

def publishXY(x, y):
    pub.publish(str(x) + ' ' + str(y))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
