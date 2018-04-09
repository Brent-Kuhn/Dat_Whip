#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os

class coneFinder:
    def __init__(self):
        rp.init_node("coneFinder",anonymous=False)
        self.pub=rp.Publisher("coneFinder",String,queue_size=10)
        rp.spin()
        cap = cv2.VideoCapture(1)
        while(cap.isOpened()):
            ret,image = cap.read()
            hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            mask = self.colorFilter(hsv)
            x,y = self.findCenter(mask)
            height, width, _ = image.shape
            self.steer(height,width,x,y)

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

        minBlue = np.array([100,75,75])
        maxBlue = np.array([120,255,255])
        maskBlue = cv2.inRange(hsv,minBlue,maxBlue)
        mask=cv2.bitwise_or(maskBlue,maskOrange)
        return mask

    def steer(self,height,width,x,y):
        y = height - y
        angle = -.34 * math.atan2(x, y) *(2/math.pi)
        speed = y / (height/2)
	print("here"+str(angle)+", "+str(speed))
        self.pub.publish(str(speed)+","+str(angle)+","+"0")

if __name__ == '__main__':
    try:
        coneFinder()
    except rp.ROSInterruptException:
	print("here")
        pass
