#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from classes.estopClass import estop

class steeringControl:
    def __init__(self):
        rp.init_node("driver",anonymous=False)
        self.time=0
        self.timeOut=60
        self.priority=0
        self.subscribeToWallCenter()
	self.subscribeToLaneCenter()
        self.subscribeToEstop()
        #self.subscribeToSerpentine()
        self.pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
        rp.spin()

    def subscribeToLaneCenter(self):
        rp.Subscriber("laneCenter",String,self.driveCallback)

    def subscribeToWallCenter(self):
        rp.Subscriber("wallCenter",String,self.wallCenterCallback)

    def subscribeToEstop(self):
        rp.Subscriber("eStop",String,self.estopCallback)

    def subscribeToSerpentine(self):
        rp.Subscriber("serpentine",String,self.serpentineCallback)

    def wallCenterCallback(self, data):
        self.driveCallback(data)

    def estopCallback(self, data):
        self.driveCallback(data)

    def serpentineCallback(self, data):
        self.driveCallback(data)

    def driveCallback(self,data):
        driveData=data.data.split(",")
        if(int(driveData[2])>self.priority):
            self.priority=int(driveData[2])
            self.drive(float(driveData[0]),float(driveData[1]))
    	elif(int(driveData[2])==self.priority and self.time<self.timeOut):
    	    self.time+=1
    	    self.drive(float(driveData[0]),float(driveData[1]))
    	else:
            self.priority-=1
            self.time=0

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
