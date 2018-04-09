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
	rate = rp.Rate(60)
        cap = cv2.VideoCapture(1)
        while(not rp.is_shutdown()):
            _, image = cap.read()
            leftImage = image[0:376,0:672]
            rightImage = image[0:376,672:1344]
            hsv = cv2.cvtColor(rightImage,cv2.COLOR_BGR2HSV)
            mask = self.colorFilter(hsv)
            x,y = self.findCenter(mask)
            if x!=0 and y!=0:
	        height, width, _ = rightImage.shape
                self.steer(height,width,x,y)
                rate.sleep()

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

    def steer(self,height,width,x,y):
        y = height - y
	x = x - int(width/2)
        angle = -.34 * math.atan2(x, y) *(2/math.pi)
        speed = y / (height/2)
        self.pub.publish(str(speed)+","+str(angle)+","+"0")

if __name__ == '__main__':
    try:
        coneFinder()
    except rp.ROSInterruptException:
        pass
