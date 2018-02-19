#!/usr/bin/python
import math
import rospy as rp
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
pub=None

def callback(data):
    # the code for returning y and x should go here
    x,y=data.split(",")
    drive(x,y)
	pass


def drive(x,y):
    global pub
    direction = (-.34) * math.atan2(y, x)/ 1.57 # Assuming pos is left and neg is right
    speed = y / 188  #change this value to the max length of a line

	drive_msg_stamped = AckermannDriveStamped()
	drive_msg = AckermannDrive()
   	drive_msg.speed = speed
    drive_msg.steering_angle = direction
    drive_msg.acceleration = 0
    drive_msg.jerk = 0
    drive_msg.steering_angle_velocity = 0
	drive_msg_stamped.drive = drive_msg
	pub.publish(drive_msg_stamped)

def main():
    global pub
    rospy.init_node("driveListen",anonymous=False)
    rospy.Subscriber("NAME_OF_GABES_PUBLISHER",String,callback)
    rp.init_node("plzWerk",anonymous = False)
	pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
    rospy.spin()

if __name__ =="__main__":
	try:
    	main()
	except rp.ROSInterruptException:
		pass
