#!/usr/bin/python
from classes.LidarHelperClass import LidarHelper
import math

GOAL_X = .5

class StateGoToSide(object):

    def error(self, lidar, zed):
        minIndex, minDistance = LidarHelper.shortestDistInRange(\
            lidar, self.direction() * -(90 + 5), self.direction() * 45)
        minAngle = LidarHelper.lidarIndexToAngle(minIndex)
        if minDistance > 2:
            return 0
        offsetX = minDistance * math.cos(math.radians(minAngle + 90 * self.direction()))

        return self.direction() * (offsetX - GOAL_X)

    def shouldChangeState(self, lidar, zed):
        LEFT_RANGE = 2
        _, minDistance = LidarHelper.shortestDistInRange(\
            lidar, self.direction() * -90 - LEFT_RANGE, self.direction() * -90 + LEFT_RANGE)
        return minDistance < .6
