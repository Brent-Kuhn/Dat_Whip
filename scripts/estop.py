#!/usr/bin/python
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy as rp
from constants import ESTOP_START, ESTOP_STOP, ESTOP_DISTANCE

class estop:
    def __init__(self):
        rp.init_node("eStop",anonymous=False)
        self.pub=rp.Publisher("eStop",String,queue_size=10)
        self.subscribeToScan()
        self.start = ESTOP_START
        self.stop = ESTOP_STOP
        self.distance = ESTOP_DISTANCE
        rp.spin()

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.processInput)

    def processInput(self,data):
        ranges=data.ranges
        ranges=ranges[self.start:self.stop]
        if min(ranges)<self.distance:
            self.pub.publish("0,0,5")

if __name__ == '__main__':
    try:
        estop()
    except rp.ROSInterruptException:
        pass
