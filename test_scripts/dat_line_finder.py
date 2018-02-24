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

DISPLAY_IMAGE = False

SHOW_ALL_STEPS = False
GAUSS_KERNEL = 5

image_height = 0

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

last_left = [0] * 4
last_right = [0] * 4

last_left_lines = []
last_right_lines = []

def resetLineHistory():
    global last_left_lines, last_right_lines
    last_left_lines = []
    last_right_lines = []
    for i in range(MOVING_AVG_SIZE):
        last_left_lines.append([0] * 4)
        last_right_lines.append([0] * 4)

last_index = 0

BLUE_HUE = 120
BLUE_HUE_THRESH = 10
BLUE_SAT_MIN = 100
BLUE_SAT_MAX = 255
BLUE_VAL_MIN = 50
BLUE_VAL_MAX = 255

def main():
    global pub
    rp.init_node('image_processor', anonymous=False)
    pub = rp.Publisher('lineCoords', String, queue_size=10)
    resetLineHistory()
    subscribeToLeft()

def subscribeToLeft():
    rp.Subscriber('zedLeft', Image, zedLeftCallback)
    rp.spin()

def zedLeftCallback(data):
    global image_height
    image = getCVImageFromData(data)
    h, w, _ = image.shape
    image_height = h
    try:
        line = lineCoordsFromImage(image)[0]
    except IndexError:
        line = []
    x, y = getXYFromLine(line,w,h)
    publishXY(x, y)

def getCVImageFromData(data):
    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

def lineCoordsFromImage(image):
    h, w, _ = image.shape
    region_of_interest_pixels = convertToPixelRegion(REGION_OF_INTEREST, w, h)
    region = region_of_interest(image, [region_of_interest_pixels])
    filtered = colorFilter(cv2.cvtColor(region,cv2.COLOR_BGR2HSV), BLUE_HUE, BLUE_HUE_THRESH, BLUE_SAT_MIN, BLUE_SAT_MAX, BLUE_VAL_MIN, BLUE_VAL_MAX)
    blur = gaussian_blur(filtered, GAUSS_KERNEL)
    can_raw = canny(blur, CANNY_LOW_THRESH, CANNY_HIGH_THRESH)
    can = cv2.cvtColor(can_raw, cv2.COLOR_GRAY2BGR)
    lines = hough_lines(can_raw, HOUGH_RHO, HOUGH_THETA, \
        HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP)

    single_line = processLines(lines)

    if DISPLAY_IMAGE:
        if lines is None or len(lines) == 0 or lines[0] == []:
            lines = [[[0, 0, 0, 0]]]
        lines = removeThatStupidDimension(lines)
        raw_hough = newLinesImage(lines, w, h)
        output = combineImages(\
            (image, filtered, can, region, raw_hough, newLinesImage(single_line, w, h)))
        showImage(output)

    return single_line

def newLinesImage(lines, w, h):
    image = np.zeros((h, w, 3), dtype=np.uint8)
    draw_lines(image, lines, thickness=5)
    return image

def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
NOTE: this is the function you might want to use as a starting point once you want to average/extrapolate the line segments you detect to map out the full extent of the lane (going from the result shown in raw-lines-example.mp4    to that shown in P1_example.mp4).

Think about things like separating line segments by their
slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
line vs. the right line.  Then, you can average the position of each of
the lines and extrapolate to the top and bottom of the lane.

This function draws `lines` with `color` and `thickness`.
Lines are drawn on the image inplace (mutates the image).
If you want to make the lines semi-transparent, think about combining
this function with the weighted_img() function below
    """

    for line in lines:
        x1,y1,x2,y2 = line
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def combineImages(images):
    a = np.concatenate((images[0], images[1], images[2]), axis=1)
    b = np.concatenate((images[3], images[4], images[5]), axis=1)
    return np.concatenate((a, b), axis=0)

def colorFilter(image, hue, hue_thresh, sat_min, sat_max, val_min, val_max):
    lower = np.array([hue - hue_thresh, sat_min, val_min])
    upper = np.array([hue + hue_thresh, sat_max, val_max])
    mask = cv2.inRange(image, lower, upper)
    return cv2.bitwise_and(image, image, mask=mask)

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

    #if False and lines is None:
    #    moving_left_avg = last_left
    #    moving_right_avg = last_right
    if lines is not None:
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
    else:
        return []

def ensureLinesGoTopToBottom(lines):
    size = len(lines)
    for i in range(size):
        [x1, y1, x2, y2] = lines[i]
        if y1 > y2:
            lines[i] = [x2, y2, x1, y1]
    return lines

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
    try:
        [topX, topY, _, _] = line
        x = topX - int(width / 2)
        y = height - topY
        return x, y
    except Exception:
        return 0, 0

def publishXY(x, y):
    global pub
    pub.publish(str(x) + ',' + str(y))

def showImage(image):
    if DISPLAY_IMAGE:
        cv2.imshow('image', image)
        if cv2.waitKey(20) & 0xFF == ord('q'):
	    pass

if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass
