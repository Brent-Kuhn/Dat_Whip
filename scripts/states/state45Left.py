#!/usr/bin/python
from states.state45 import State45

class State45Left(State45):
    def error(self, lidar, zed, imu):
        return 1 # Turn left, indiscriminately
        # TODO don't hit the cone if possible

    def nextState(self, lidar, zed, imu):
        return 'StateCircleRight'
