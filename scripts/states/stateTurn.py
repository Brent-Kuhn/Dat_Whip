#!/usr/bin/python
from classes.LidarHelperClass import LidarHelper
import rospy as rp
from std_msgs.msg import String

def findMainObject(zed):
    # TODO replace with Brent's awesome method
    return 'cone'

class StateTurn(object):

    def shouldChangeState(self, lidar, zed):
        return self.somethingIsInFront(lidar, zed)

    def somethingIsInFront(self, lidar, zed):
        self.zedObject = findMainObject(zed)
        return self.lidarSomethingIsInFront(lidar) \
            and self.zedSomethingIsInFront()

    def lidarSomethingIsInFront(self, lidar):
        FRONT_RANGE = 2
        _, minDistance = LidarHelper.shortestDistInRange(lidar, -FRONT_RANGE, FRONT_RANGE)
        return minDistance < .7

    def zedSomethingIsInFront(self):
        return self.zedObject != ''

    def zedConeIsInFront(self):
        return self.zedObject == 'cone'
