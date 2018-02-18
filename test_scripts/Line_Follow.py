#!/usr/bin/python
import cv2
import math
import rospy as rp
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


def callback(data):
    bridge = CvBridge()
    img=bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # the code for returning y and x should go here
    if cv2.waitKey(20) & 0xFF == ord('q'):
	pass


def Drive(coords):
    direction = (-1) * math.atan2(coords[0], coords[1])/ 1.57 # Assuming pos is left and neg is right
    speed = coords[1] / 50  #change this value to the max length of a line
	rp.init_node("pleaseWork",anonymous = False)
	pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
	rate=rp.Rate(60)
	drive_msg_stamped = AckermannDriveStamped()
	drive_msg = AckermannDrive()
       	drive_msg.speed = speed
        drive_msg.steering_angle = direction
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
	drive_msg_stamped.drive = drive_msg

	pub.publish(drive_msg_stamped)
	rate.sleep()

def main():
    while True:
        rospy.init_node("leftListen",anonymous=False)
        rospy.Subscriber("zedLeft",Image,callback)
        Drive()
        rospy.spin()

if __name__ =="__main__":
	try:
    	main()
	except rp.ROSInterruptException:
		pass
