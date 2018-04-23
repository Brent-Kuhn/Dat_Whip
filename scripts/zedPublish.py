#!/usr/bin/python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    global cap
    bridge = CvBridge()
    rightPub = rospy.Publisher('zedImage', Image, queue_size=1)
    rospy.init_node('camera', anonymous=False)
    rate = rospy.Rate(60)
    cap = cv2.VideoCapture(1)

    while(not rospy.is_shutdown()):
        ret,img = cap.read()

        rightMsg = bridge.cv2_to_imgmsg(img, encoding='bgr8')

        rightPub.publish(rightMsg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
