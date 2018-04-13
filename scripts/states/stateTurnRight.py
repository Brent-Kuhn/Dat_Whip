#!/usr/bin/python
from states.stateTurn import StateTurn

class StateTurnRight(StateTurn):
    def error(self, lidar, zed):
        return -1 # Turn right, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed):
        return 'StateGoToRight' if self.zedConeIsInFront() else 'StateCircleLeft'
