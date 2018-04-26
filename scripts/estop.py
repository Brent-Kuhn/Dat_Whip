#!/usr/bin/python
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy as rp
from constants import ESTOP_FRONT_START, ESTOP_FRONT_STOP, ESTOP_FRONT_DISTANCE,ESTOP_RIGHT_START,ESTOP_RIGHT_STOP,ESTOP_LEFT_START,ESTOP_LEFT_STOP,ESTOP_SIDE_DISTANCE

STOP_DURATION = 30
BACKUP_DURATION = 30

class estop:
    def __init__(self):
        rp.init_node("eStop",anonymous=False)
        self.pub=rp.Publisher("eStop",String,queue_size=10)

        self.stoppedTime = 0
        self.backupTime = BACKUP_DURATION

        self.subscribeToScan()
        self.frontDistance = ESTOP_FRONT_DISTANCE
        self.sideDistance = ESTOP_SIDE_DISTANCE
        rp.spin()

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.processInput)

    def processInput(self,data):
        ranges=data.ranges

        if self.shouldStop(ranges):
            if self.stoppedTime > STOP_DURATION and self.backupTime > 0:
                self.pub.publish("-1,0,100")
                self.backupTime -= 1
            else:
                self.pub.publish("0,0,100")
                self.stoppedTime += 1
        else:
            self.stoppedTime = 0
            self.backupTime = BACKUP_DURATION

    def shouldStop(self, ranges):
        return min(ranges[ESTOP_FRONT_START:ESTOP_FRONT_STOP])<self.frontDistance \
            or min(ranges[ESTOP_RIGHT_START:ESTOP_RIGHT_STOP])<self.sideDistance \
            or min(ranges[ESTOP_LEFT_START:ESTOP_LEFT_STOP])<self.sideDistance

if __name__ == '__main__':
    try:
        estop()
    except rp.ROSInterruptException:
        pass
