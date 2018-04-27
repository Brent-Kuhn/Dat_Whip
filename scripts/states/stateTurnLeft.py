#!/usr/bin/python
from states.stateTurn import StateTurn

class StateTurnLeft(StateTurn):
    def error(self, lidar, zed, imu):
        return 1 # Turn left, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed, imu):
        return 'StateGoToLeft' if self.zedConeIsInFront() else 'State45Left'
