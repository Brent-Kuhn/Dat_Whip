#!/usr/bin/python
from states.stateTurn import StateTurn

class StateTurnLeft(StateTurn):
    def error(self, lidar, zed):
        return 1 # Turn left, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed):
        return 'StateGoToLeft' if self.zedConeIsInFront() else 'StateCircleRight'
