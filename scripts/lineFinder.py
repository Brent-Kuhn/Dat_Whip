#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from classes.lineDriveClass import LineDrive
from classes.estopClass import estop

class steeringControl:
    def __init__(self):
        rp.init_node("lineSteer",anonymous=False)
        self.lineDriver=LineDrive(30)
        self.estop=estop(30,240,841)
        self.subscribeToLine()
        #self.subscribeToScan()
        self.pub=rp.Publisher("/vesc/ackermann_cmd_mux/input/navigation",AckermannDriveStamped,queue_size=10)
        rp.spin()

    def subscribeToLine(self):
        rp.Subscriber("lineCoords",String,self.camCallback)

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.scanCallback)

    def camCallback(self,lineData):
        try:
            speed,angle=self.lineDriver.processLine(lineData.data)
            self.drive(speed,angle)
        except Exception:
            pass

    def scanCallback(self,data):
        try:
            speed,angle=self.estop.processInput(data.ranges)
            self.drive(speed,angle)
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
