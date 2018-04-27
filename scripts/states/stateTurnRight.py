#!/usr/bin/python
from states.stateTurn import StateTurn

class StateTurnRight(StateTurn):
    def error(self, lidar, zed, imu):
        return -1 # Turn right, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed, imu):
        return 'StateGoToRight' if self.zedConeIsInFront() else 'State45Right'
