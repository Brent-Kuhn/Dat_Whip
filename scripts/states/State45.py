#!/usr/bin/python

class State45(object):

    def __init__(self):
        # TODO set orientation

    def shouldChangeState(self, lidar, zed):
        return abs(TODO.getOrientationZ()) > 45
