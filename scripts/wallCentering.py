#!/usr/bin/python
import rospy as rp
from sensor_msgs.msg import LaserScan
from classes.lidarSteerManagerClass import LidarSteerManager
from std_msgs.msg import String

class WallCentering:
    def __init__(self):
        rp.init_node("lidarSteer",anonymous=False)
        self.lidarSteer=LidarSteerManager()

        self.subscribeToScan()
        self.pub=rp.Publisher("/wallCenter",String,queue_size=10)
        rp.spin()

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.scanCallback)

    def scanCallback(self,data):
        steerDirection,speed,priority=self.lidarSteer.steer(data.ranges)
        self.pub.publish(str((.5+speed)*1.5)+","+str(steerDirection)+","+priority)

if __name__ == '__main__':
    try:
        WallCentering()
    except rp.ROSInterruptException:
        pass
