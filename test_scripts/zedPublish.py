#!/usr/bin/python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    global cap
    bridge = CvBridge()
    leftPub = rospy.Publisher('zedLeft', Image, queue_size=10)
    rightPub = rospy.Publisher('zedRight', Image, queue_size=10)
    rospy.init_node('camera', anonymous=False)
    rate = rospy.Rate(60)
    cap = cv2.VideoCapture(1)

    while(cap.isOpened()):
        ret,img = cap.read()

        leftImg=img[0:376,0:672]
        rightImg=img[0:376,672:1344]

        leftMsg = bridge.cv2_to_imgmsg(leftImg, encoding="passthrough")
        rightMsg = bridge.cv2_to_imgmsg(rightImg, encoding="passthrough")

        leftPub.publish(leftMsg)
        rightPub.publish(rightMsg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
