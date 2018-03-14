#!/usr/bin/python
import rospy as rp
from sensor_msgs.msg import LaserScan
from classes.lidarSteerManagerClass import LidarSteerManager

class WallCentering:
    def __init__(self):
        rp.init_node("lidarSteer",anonymous=False)
        self.lidarSteer=LidarSteerManager()

        self.subscribeToScan()
        self.pub=rp.Publisher("/wallCenter",AckermannDriveStamped,queue_size=10)
        rp.spin()

    def subscribeToScan(self):
        rp.Subscriber("scan",LaserScan,self.scanCallback)

    def scanCallback(self,data):
        ranges=[[]]
        try:
            ranges[0]=data.ranges[0:360]
            ranges[1]=data.ranges[720:1080]
            steerDirection=self.lidarSteer.steer(ranges)
            self.pub.publish("1,"+steerDirection,",1")
        except Exception:
            pass

if __name__ == '__main__':
    try:
        WallCentering()
    except rp.ROSInterruptException:
        pass
