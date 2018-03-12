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
        #self.subscribeToWallCenter()
        self.subscribeToTest()
        self.subscribeToEstop()
        self.pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
        rp.spin()

    def subscribeToWallCenter(self):
        rp.Subscriber("wallCenter",String,self.driveCallback)

    def subscribeToTest(self):
        rp.Subscriber("testDrive",String,self.driveCallback)

    def subscribeToEstop(self):
        rp.Subscriber("eStop",LaserScan,self.driveCallback)

    def driveCallback(self,data):
        driveData=data.data.split(",")
        try:
            if(driveData[2]>self.priority and self.time<=self.timeOut):
                self.time+=1
                self.priority=driveData[2]
                self.drive(driveData[0],driveData[1])
            else:
                self.priority=0
                self.time=0
        except Exception:
            pass

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