#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String

def driveTest():
	rp.init_node("testDrive",anonymous = False)
	pub=rp.Publisher("testDriver",String,queue_size=10)
	rate=rp.Rate(60)
	while True:
		pub.publish(".8,1,0")
		rate.sleep()

if __name__ =="__main__":
	try:
		driveTest()
	except rp.ROSInterruptException:
		pass
