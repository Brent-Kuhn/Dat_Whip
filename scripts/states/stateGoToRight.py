#!/usr/bin/python
from classes.LidarHelperClass import LidarHelper
from states.stateGoToSide import StateGoToSide

class StateGoToRight(StateGoToSide):

    def direction(self):
        return 1

    def nextState(self, lidar, zed, imu):
        return 'StateTurnLeft'
