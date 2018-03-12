#!/usr/bin/python
import numpy as np

### FLAGS ###
### Changing these change major functions of the car ###

# Whether to display images or not (should be FALSE when not plugged in to a TV)
DISPLAY_IMAGE = False

# Whether to send images over network (leave FALSE unless specifically debugging)
STREAM_IMAGE = False

#
#
#

### CV CONFIGURATIONS ###
### Configure these to modify how the line is processed ###

# Size of the gaussian blur radius
GAUSS_KERNEL = 5

# Canny edge detection parameters
CANNY_LOW_THRESH = 50
CANNY_HIGH_THRESH = 75

# Region to crop line-finding image, as percentages of the image size
REGION_OF_INTEREST = np.array(\
    [[0, 0.4], [1, 0.4], [1, 1], [0, 1]], np.float32)

# Hough line parameters... I don't honestly understand what these do
HOUGH_RHO = 1
HOUGH_THETA = np.pi / 180
HOUGH_THRESH = 20
HOUGH_MIN_LEN = 50
HOUGH_MAX_GAP = 30

# Define the exact blue color we are looking for for line following
BLUE_HUE = 120
BLUE_HUE_THRESH = 17
BLUE_SAT_MIN = 100
BLUE_SAT_MAX = 255
BLUE_VAL_MIN = 50
BLUE_VAL_MAX = 255

#
#
#

### LINE PROCESSING ###
### Change to modify how the hough lines are condensed to a single line ###

# Minimum slope, any lines underneath this slope will be ignored
SLOPE_MIN_THRESH = .1

# How many previous lines to keep for the moving average
HISTORY_SIZE = 3

#
#
#
### PORTS ###
### I really want all ports to be put here ###

# Debug line finding stream port #
DEBUG_STREAM_PORT = 8081
