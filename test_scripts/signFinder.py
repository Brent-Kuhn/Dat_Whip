#!/usr/bin/python
import rospy as rp
import cv2
import rospy as rp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import os

class coneFinder:
    def __init__(self):
        self.bridge = CvBridge() 
	rp.init_node('rightImage', anonymous=False)
	rp.Subscriber("zedImage", Image, self.zedCallback)
	rp.spin()

    def zedCallback(self, zed):
	zed = self.bridge.imgmsg_to_cv2(zed)
	cv2.imshow("zedRight", zed)

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
        redMin = np.array([0,0,100])
        redMax = np.array([10,255,255])
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
