#!/usr/bin/python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    img=bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    cv2.imshow("data",img)
    if cv2.waitKey(20) & 0xFF == ord('q'):
	pass

def main():
    rospy.init_node("leftListen",anonymous=False)
    rospy.Subscriber("zedLeft",Image,callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
