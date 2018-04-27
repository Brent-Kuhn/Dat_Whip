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
from random import random

REDL_MIN = np.array([0,188,83])
REDL_MAX = np.array([04,255,139])

REDH_MIN = np.array([177,188,83])
REDH_MAX = np.array([180,255,139])

class stopSign:
    def __init__(self):
        rp.init_node("stopSign",anonymous=False)
        self.pub=rp.Publisher("eStop",String,queue_size=10)
        self.bridge = CvBridge()
        self.subscribeToImage()
        rp.spin()

    def subscribeToImage(self):
        rp.Subscriber('zedImage', Image, self.zedCallback)

    def zedCallback(self, data):
        image = self.bridge.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        REDL_FINAL = cv2.inRange(hsv, REDL_MIN, REDL_MAX)
        REDH_FINAL = cv2.inRange(hsv, REDH_MIN, REDH_MAX)
        RED_FINAL = cv2.bitwise_or(REDL_FINAL, REDH_FINAL)
        mask = cv2.bitwise_and(image, image, mask=RED_FINAL)
        x, y, area = self.findCenter(RED_FINAL)
        # Remove coments for testing
        #print(area)
        if x != 0 and y != 0 and area > (410353/2):
            height, width, _ = image.shape
            angle = 0
            speed = 0
            while(True):
                self.pub.publish(str(speed)+","+str(angle)+","+"100")
            print("stop", area)

    def findCenter(self,mask):
        blur = cv2.GaussianBlur(mask,(5,5),0)
        closed = cv2.morphologyEx(blur,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        contours = cv2.findContours(closed,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        M = cv2.moments(contours[0])
        if(M["m00"]==0):
            return 0,0,0
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX,cY,M["m00"]

    def colorFilter(self, hsv, colorMin, colorMax):
        return cv2.inRange(hsv, colorMin, colorMax)

if __name__ == '__main__':
    try:
        stopSign()
    except rp.ROSInterruptException:
        pass
