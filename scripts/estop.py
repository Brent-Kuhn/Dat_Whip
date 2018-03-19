#!/usr/bin/python
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy as rp
from constants import ESTOP_FRONT_START, ESTOP_FRONT_STOP, ESTOP_FRONT_DISTANCE,ESTOP_RIGHT_START,ESTOP_RIGHT_STOP,ESTOP_LEFT_START,ESTOP_LEFT_STOP,ESTOP_SIDE_DISTANCE

class estop:
    def __init__(self):
        rp.init_node("eStop",anonymous=False)
        self.pub=rp.Publisher("eStop",String,queue_size=10)
        self.subscribeToScan()
        self.frontDistance = ESTOP_FRONT_DISTANCE
        self.sideDistance = ESTOP_SIDE_DISTANCE
        rp.spin()

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.processInput)

    def processInput(self,data):
        ranges=data.ranges
        if min(ranges[ESTOP_FRONT_START:ESTOP_FRONT_STOP])<self.frontDistance or min(ranges[ESTOP_RIGHT_START:ESTOP_RIGHT_STOP])<self.sideDistance or  min(ranges[ESTOP_LEFT_START:ESTOP_LEFT_STOP])<self.sideDistance:
            self.pub.publish("0,0,5")

if __name__ == '__main__':
    try:
        estop()
    except rp.ROSInterruptException:
        pass
