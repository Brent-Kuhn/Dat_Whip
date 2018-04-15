#!/usr/bin/python
from states.State45 import State45

class State45Right(State45):
    def error(self, lidar, zed):
        return -1 # Turn right, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed):
        return 'StateCircleLeft'
