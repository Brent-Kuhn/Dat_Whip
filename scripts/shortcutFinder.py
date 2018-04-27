#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from classes.LidarHelperClass import LidarHelper
from constants import STREAM_IMAGE

GREEN_MIN = np.array([67,151,22])
GREEN_MAX = np.array([87,226,101])

LOOK_FOR_SHORTCUT_LENGTH = 0.6
HOLE_DEPTH = 1.5
SEARCH_STEP = 2
DIR_RIGHT = -SEARCH_STEP
DIR_LEFT = SEARCH_STEP

class shortcutFinder:
    def __init__(self):
        rp.init_node("shortcutFinder",anonymous=False)
        self.pub=rp.Publisher("shortcutFinder",String,queue_size=10)
        if STREAM_IMAGE:
            self.debugPub = rp.Publisher('debugShortcut', Image, queue_size=1)
        self.bridge = CvBridge()
        # self.subscribeToScan()
        self.subscribeToImage()
        rp.spin()

    def subscribeToImage(self):
        rp.Subscriber('zedImage', Image, self.zedCallback)

    def zedCallback(self, data):
        image = self.bridge.imgmsg_to_cv2(data)
        leftImage = image[0:376,0:672]
        hsv = cv2.cvtColor(leftImage, cv2.COLOR_BGR2HSV)
        mask = self.colorFilter(hsv, GREEN_MIN, GREEN_MAX)
        x, y, area = self.findCenter(mask)
        if STREAM_IMAGE:
            debugImage = cv2.bitwise_and(leftImage, leftImage, mask=mask)
            debugImage = cv2.circle(debugImage, (x, y), 4, GREEN_MAX)
            self.debugPub.publish(self.bridge.cv2_to_imgmsg(debugImage, encoding='bgr8'))
        if x != 0 and y != 0 and area > 1000:
            height, width, _ = leftImage.shape
            print('and at last I see the sign %.2f, %.2f, %.2f' % (x, y, area))
            self.steerTowardSignOrIntoHole(height, width, x, y)

    def findCenter(self,mask):
        blur = cv2.GaussianBlur(mask,(5,5),0)
        closed = cv2.morphologyEx(blur,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        contours = cv2.findContours(closed,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        M = cv2.moments(contours[0])
        if(M["m00"]==0):
            return 0,0,0
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX,cY, M["m00"]

    def colorFilter(self, hsv, colorMin, colorMax):
        return cv2.inRange(hsv, colorMin, colorMax)

    def steerTowardSignOrIntoHole(self, height, width, x, y):
        self.steer(height, width, x, y)

    def steer(self,height,width,x,y):
        y = height - y
        x = x - int(width/4)
        angle = -.34 * math.atan2(x, y) *(2/math.pi)
        speed = y / (height/2)
        self.pub.publish(str(speed)+","+str(angle)+","+"5")

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.lidarCallback)

    def lidarCallback(self, data):
        self.lidar = data.ranges
        lidar = self.lidar
        minIndex, minDistance = LidarHelper.shortestDistInRange(lidar, -25, 25)
        if minDistance < LOOK_FOR_SHORTCUT_LENGTH:
            angleLeft = self.findHoleAngleLeft(lidar, minIndex)
            angleRight = self.findHoleAngleRight(lidar, minIndex)
            print('whole angle at %.2f, %.2f' % (angleLeft, angleRight))

    def findHoleAngleRight(self, lidar, minIndex):
        return self.findHoleAngle(lidar, minIndex, DIR_RIGHT)

    def findHoleAngleLeft(self, lidar, minIndex):
        return self.findHoleAngle(lidar, minIndex, DIR_LEFT)

    def findHoleAngle(self, lidar, minIndex, dir):
        index = minIndex
        try:
            #TODO restict ranges a bit more
            while lidar[index] < HOLE_DEPTH:
                index += dir
            startHoleIndex = index
            while lidar[index] >= HOLE_DEPTH:
                index += dir
            endHoleIndex = index
            holeIndex = self.mid(startHoleIndex, endHoleIndex)
            return LidarHelper.lidarIndexToAngle(holeIndex)
        except IndexError:
            return -1 # It's not on this side

    def mid(self, a, b):
        return (a + b) / 2

if __name__ == '__main__':
    try:
        shortcutFinder()
    except rp.ROSInterruptException:
        pass
