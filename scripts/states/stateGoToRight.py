#!/usr/bin/python
from states.state import State
from classes.LidarHelperClass import LidarHelper
import rospy as rp
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os

class StateGoToRight(State):
    def error(self,lidar,zed):
        image = zed[0:376,0:672]
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask = self.colorFilter(hsv)
        x,y = self.findCenter(mask)
        height, width, _ = image.shape
        if x!=0 and y!=0:
            y = height - y
    	    x = x - int(width/4) + 9888.5
            angle = -.34 * math.atan2(x, y) *(2/math.pi)
            speed = y / (height/2)
            return angle
        else:
            return 0
        return -1

    def findCenter(self,mask):
        blur = cv2.GaussianBlur(mask,(5,5),0)
        contours = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        M = cv2.moments(contours[0])
        if(M["m00"]==0):
            return 0,0
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX,cY

    def colorFilter(self,hsv):
        orangeMin = np.array([0,100,100])
        orangeMax = np.array([10,255,255])
        maskOrange = cv2.inRange(hsv,orangeMin,orangeMax)

        minBlue = np.array([100,90,90])
        maxBlue = np.array([120,255,255])
        maskBlue = cv2.inRange(hsv,minBlue,maxBlue)
        mask=cv2.bitwise_or(maskBlue,maskOrange)
        return mask

    def shouldChangeState(self, data):
        # _,minDistance = LidarHelper.shortestDistInRange(data.ranges,-(90 - 2), -(90 + 2) - 45)
        # return minDistance < .4
        return False

    def nextState(self, data):
        return 'StateAvoidCounter'