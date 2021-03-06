#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from constants import STREAM_IMAGE

BLUE_MIN = np.array([90,100,80])
BLUE_MAX = np.array([108,248,255])
BRIGHT_MIN = np.array([245,245,245])
BRIGHT_MAX = np.array([255,255,255])

class LaneDriver:
	def __init__(self):
		rp.init_node("laneCenter",anonymous=False)
		self.pub=rp.Publisher("laneCenter",String,queue_size=10)
		self.bridge = CvBridge()
		if STREAM_IMAGE:
			self.debugPub = rp.Publisher('zedDebug', Image, queue_size=1)
			self.debugCount = 0
			blackImage = np.zeros((376, 672, 3), np.uint8)
			self.debugImages = [blackImage, blackImage, blackImage, blackImage]
		rp.Subscriber("zedImage",Image,self.zedCallback)
		rp.spin()

	def zedCallback(self,data):
		 image = self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
		 leftImage = image[0:376,0:672]
		 rightImage = image[0:376,672:1344]
		 leftSpeed,leftAngle,leftPriority = self.getSteering(leftImage)
		 rightSpeed,rightAngle, rightPriority = self.getSteering(rightImage)
		 finalSpeed = (leftSpeed+rightSpeed)/2
		 finalAngle = (leftAngle+rightAngle)/2
		 priority = leftPriority if leftPriority > rightPriority else rightPriority
		 if not (finalSpeed == 0 or finalAngle == 0):
			 self.pub.publish(str(1+finalSpeed)+","+str(finalAngle)+","+str(priority))

	def getSteering(self,image):
		current=image[255:image.shape[0],0:image.shape[1]]
		future=image[134:255,0:image.shape[1]]
		currentSpeed,currentAngle,currentArea = self.processImage(current)
		futureSpeed,futureAngle,futureArea = self.processImage(future)
		FUTURE_WEIGHT = 1.5
		CURRENT_WEIGHT = 1#1 - FUTURE_WEIGHT
		totalSpeed = (futureSpeed * FUTURE_WEIGHT) + (currentSpeed * CURRENT_WEIGHT)
		totalAngle = (futureAngle * FUTURE_WEIGHT) + (currentAngle * CURRENT_WEIGHT)
		priority = self.calcPriorityFromAreas(currentArea, futureArea)
		return totalSpeed,totalAngle,priority

	def calcPriorityFromAreas(self, currentArea, futureArea):
		area = currentArea + futureArea
		if area > 10455730.0 * 2:
			priority = "4"
		else:
			priority = "2"
		return priority

	def processImage(self,image):
		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		mask = self.maskImage(image, hsv)
		x,y,area = self.findCenter(mask)
		if STREAM_IMAGE:
			debugImage = cv2.bitwise_and(image, image, mask=mask)
			debugImage = cv2.circle(debugImage, (x, y), 4, BLUE_MAX)
			self.debugImages[self.debugCount % 4] = debugImage
			if self.debugCount % 4 == 0 and self.debugCount > 0:
				# 0 = BL, 1 = TL, 2 = BR, 3 = TR
				debugImageL = np.concatenate((self.debugImages[1], self.debugImages[0]), axis=0)
				debugImageR = np.concatenate((self.debugImages[3], self.debugImages[2]), axis=0)
				debugImageBoth = np.concatenate((debugImageL, debugImageR), axis=1)
				self.debugPub.publish(self.bridge.cv2_to_imgmsg(debugImageBoth, encoding='bgr8'))
			self.debugCount += 1
		if (x == 0 and y == 0):
			return 0,0,0
		height, width, _ = image.shape
		return self.steer(height,width,x,y,area)

	def findCenter(self,mask):
		blur = cv2.GaussianBlur(mask,(5,5),0)
		closed = cv2.morphologyEx(blur,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
		contours = cv2.findContours(closed,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		M = cv2.moments(contours[0])
		if(M["m00"]==0):
			return 0,0,0
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		return cX,cY,M["m00"]

	def maskImage(self,image, hsv):
		maskBlue = cv2.inRange(hsv, BLUE_MIN, BLUE_MAX)
		maskBright = cv2.inRange(image,BRIGHT_MIN,BRIGHT_MAX)
		cv2.bitwise_or(maskBright,maskBlue,maskBright)
		return maskBright

	def steer(self,height,width,x,y,area):
		y = height - y
		x = x - (width/2)
		angle = -.34 * math.atan2(x, y) *(2/math.pi)
		speed = area / 100000000 * 4#((y * 1.0)/ height) * 2
		return speed,angle,area

if __name__ == '__main__':
	try:
		LaneDriver()
	except rp.ROSInterruptException:
		pass
