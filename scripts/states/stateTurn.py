#!/usr/bin/python
from classes.LidarHelperClass import LidarHelper
import rospy as rp
from std_msgs.msg import String
from object_detection.object_detection_images import findMainObject

class StateTurn(object):

    def shouldChangeState(self, lidar, zed, imu):
        return self.somethingIsInFront(lidar, zed)

    def somethingIsInFront(self, lidar, zed):
        if not self.lidarSomethingIsInFront(lidar):
            return False
        self.zedObject = findMainObject(zed)
        return self.zedSomethingIsInFront()

    def lidarSomethingIsInFront(self, lidar):
        FRONT_RANGE = 2
        _, minDistance = LidarHelper.shortestDistInRange(lidar, -FRONT_RANGE, FRONT_RANGE)
        return minDistance < 2.3

    def zedSomethingIsInFront(self):
        return self.zedObject != ''

    def zedConeIsInFront(self):
        return self.zedObject == 'cone'
