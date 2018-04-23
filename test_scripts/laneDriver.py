#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneDriver:
    def __init__(self):
        rp.init_node("laneCenter",anonymous=False)
        self.pub=rp.Publisher("laneCenter",String,queue_size=10)
        self.bridge = CvBridge()
	rp.Subscriber("zedImage",Image,self.zedCallback)
	rp.spin()

    def zedCallback(self,data):
         image = self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
         leftImage = image[0:376,0:672]
         rightImage = image[0:376,672:1344]
         leftSpeed,leftAngle = self.getSteering(leftImage)
         rightSpeed,rightAngle = self.getSteering(rightImage)
         finalSpeed = (leftSpeed+rightSpeed)/2
         finalAngle = (leftAngle+rightAngle)/2
         if not (finalSpeed == 0 and finalAngle == 0):
             self.pub.publish(str(finalSpeed)+","+str(finalAngle)+","+"1")

    def getSteering(self,image):
        current=image[255:image.shape[0],0:image.shape[1]]
        future=image[134:255,0:image.shape[1]]
        currentSpeed,currentAngle = self.processImage(current)
        futureSpeed,futureAngle = self.processImage(future)
        totalSpeed = (futureSpeed * .8) + (currentSpeed * .2)
        totalAngel = (futureAngle * .8) + (currentAngle * .2)
        return totalSpeed,totalAngel

    def processImage(self,image):
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask = self.maskImage(hsv)
        x,y = self.findCenter(mask)
        if (x == 0 and y == 0):
            return 0,0
        height, width, _ = image.shape
        return self.steer(height,width,x,y)

    def findCenter(self,mask):
        blur = cv2.GaussianBlur(mask,(5,5),0)
        contours = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        M = cv2.moments(contours[0])
        if(M["m00"]==0):
            return 0,0
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX,cY

    def maskImage(self,hsv):
        minBlue = np.array([56,0,87])
        maxBlue = np.array([120,155,255])
        maskBlue = cv2.inRange(hsv,minBlue,maxBlue)
        return maskBlue

    def steer(self,height,width,x,y):
        y = height - y
        x = x - (width/2)
        angle = -.34 * math.atan2(x, y) *(2/math.pi)
        speed = ((y * 1.0)/ height) * 2
        return speed,angle

if __name__ == '__main__':
    try:
        LaneDriver()
    except rp.ROSInterruptException:
        pass
