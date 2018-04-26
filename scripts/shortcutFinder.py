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

ORGANGE_MIN = np.array([0,100,100])
ORANGE_MAX = np.array([10,255,255])
BLUE_MIN = np.array([100,90,90])
BLUE_MAX = np.array([120,255,255])

GREEN_MIN = np.array([25,29,0])
GREEN_MAX = np.array([84,255,119])

class shortcutFinder:
    def __init__(self):
        rp.init_node("shortcutFinder",anonymous=False)
        self.pub=rp.Publisher("shortcutFinder",String,queue_size=10)
        self.bridge = CvBridge()
        self.subscribeToScan()
        self.subscribeToImage()
        rp.spin()

    def subscribeToImage(self):
        rp.Subscriber('zedImage', Image, self.zedCallback)

    def zedCallback(self, data):
        image = self.bridge.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self.colorFilter(hsv, GREEN_MIN, GREEN_MAX)
        x, y = self.findCenter(mask)
        if x != 0 and y != 0:
            height, width, _ = image.shape
            self.steerTowardSignOrIntoHole(height, width, x, y)

    def findCenter(self,mask):
        blur = cv2.GaussianBlur(mask,(5,5),0)
        contours = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        M = cv2.moments(contours[0])
        if(M["m00"]==0):
            return 0,0
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX,cY

    def colorFilter(self, hsv, colorMin, colorMax):
        return cv2.inRange(hsv, colorMin, colorMax)

    def steerTowardSignOrIntoHole(self, height, width, x, y):
        self.steer(height, width, x + 3, y)

    def steer(self,height,width,x,y):
        y = height - y
        x = x - int(width/2)
        angle = -.34 * math.atan2(x + 1, y) *(2/math.pi)
        speed = y / (height/2)
        self.pub.publish(str(speed)+","+str(angle)+","+"5")

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.lidarCallback)

    def lidarCallback(self, data):
        self.lidar = data.ranges

if __name__ == '__main__':
    try:
        shortcutFinder()
    except rp.ROSInterruptException:
        pass
