#!/usr/bin/python
from states.State45 import State45

class State45Left(State45):
    def error(self, lidar, zed):
        return 1 # Turn left, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed):
        return 'StateCircleRight'
