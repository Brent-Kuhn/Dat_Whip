#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from classes.estopClass import estop

class steeringControl:
	def __init__(self):
		rp.init_node("driver",anonymous=False)
		self.time=0
		self.timeOut=10
		self.priority=0
		self.subscribeToWallCenter()
		self.subscribeToLaneCenter()
		self.subscribeToEstop()
		self.subscribeToShortcut()
		#self.subscribeToSerpentine()
		self.pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
		rp.spin()

	def subscribeToLaneCenter(self):
		rp.Subscriber("laneCenter",String,self.laneCenterCallback)

	def subscribeToWallCenter(self):
		rp.Subscriber("wallCenter",String,self.wallCenterCallback)

	def subscribeToEstop(self):
		rp.Subscriber("eStop",String,self.estopCallback)

	def subscribeToSerpentine(self):
		rp.Subscriber("serpentine",String,self.serpentineCallback)

	def subscribeToShortcut(self):
		rp.Subscriber('shortcutFinder', String, self.shortcutCallback)

	def laneCenterCallback(self, data):
		self.driveCallback(data, 'laneCenter')

	def wallCenterCallback(self, data):
		self.driveCallback(data, 'wallCenter')

	def estopCallback(self, data):
		self.driveCallback(data, 'estop')

	def serpentineCallback(self, data):
		self.driveCallback(data, 'serpentine')

	def shortcutCallback(self, data):
		self.driveCallback(data, 'shortcut')

	def driveCallback(self, data, name):
		driveData=data.data.split(",")
		newPriority = int(driveData[2])
		if newPriority > self.priority:
			self.priority = newPriority
		if newPriority == self.priority:
			self.time = 0
			# print(name + ' with a priority of ' + str(self.priority))
			self.drive(float(driveData[0]),float(driveData[1]))
		if newPriority < self.priority:
			self.time += 1
			if self.time > self.timeOut:
				self.time = 0
				self.priority = 0

	def drive(self,speed,angle):
		drive_msg_stamped = AckermannDriveStamped()
		drive_msg = AckermannDrive()
		drive_msg.speed = speed
		drive_msg.steering_angle = angle
		drive_msg.acceleration = 0
		drive_msg.jerk = 0
		drive_msg.steering_angle_velocity = 0
		drive_msg_stamped.drive = drive_msg
		self.pub.publish(drive_msg_stamped)

if __name__ == '__main__':
	try:
		steeringControl()
	except rp.ROSInterruptException:
		pass
