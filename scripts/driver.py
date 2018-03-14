#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from classes.estopClass import estop

class steeringControl:
    def __init__(self):
        rp.init_node("driver",anonymous=False)
        self.time=0
        self.timeOut=12000
        self.priority=0
        #self.subscribeToWallCenter()
        self.subscribeToTest()
        self.subscribeToEstop()
        self.pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
        rp.spin()

    def subscribeToWallCenter(self):
        rp.Subscriber("wallCenter",String,self.driveCallback)

    def subscribeToTest(self):
        rp.Subscriber("testDriver",String,self.driveCallback)

    def subscribeToEstop(self):
        rp.Subscriber("eStop",String,self.driveCallback)

    def driveCallback(self,data):
        driveData=data.data.split(",")
        if(int(driveData[2])>=self.priority):
	    print(driveData)
            self.time+=1
            self.priority=int(driveData[2])
            self.drive(float(driveData[0]),float(driveData[1]))

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
