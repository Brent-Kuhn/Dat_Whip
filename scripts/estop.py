#!/usr/bin/python
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy as rp

class estop:
    def __init__(self,attempts,start,stop):
        rp.init_node("eStop",anonymous=False)
        self.pub=rp.Publisher("driver",String,queue_size=10)
        self.subscribeToScan()
        self.maxAttempts = attempts
        self.attempts = 0
        self.start = start
        self.stop = stop

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.processInput)

    def processInput(self,data):
        ranges=data.ranges
        ranges=ranges[self.start:self.stop]
        if min(ranges)<.1778 and self.attempts<self.maxAttempts:
            self.attempts+=1
            self.pub.publish("0,0,5")
        elif min(ranges)>.1778:
            self.attempts=0

if __name__ == '__main__':
    try:
        estop()
    except rp.ROSInterruptException:
        pass
