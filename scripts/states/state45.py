#!/usr/bin/python
import math
from time import time

class State45(object):

    def __init__(self):
        self.startOrientationWasSet = False

    def shouldChangeState(self, lidar, zed, imu):
        if not self.startOrientationWasSet:
            self.startOrientationWasSet = True
            # self.startOrientation = self.verticalOrientation(imu)
            self.startOrientation = time()
        # offsetOrient = self.verticalOrientation(imu) - self.startOrientation
        # print('Turned %d degrees' % (offsetOrient * 180))
        # return abs(offsetOrient) > (67.5 / 180)
        return time() > self.startOrientation + 1 # one whole second

    # def verticalOrientation(self, imu):
    #     x, y, z = self.quatToXYZ(imu.orientation)
    #     return y
    #
    # def quatToXYZ(self, quat):
    #     angle = 2 * math.acos(quat.w)
    #     qw2 = quat.w * quat.w
    #     denom = math.sqrt(1 - qw2)
    #     x = quat.x / denom
    #     y = quat.y / denom
    #     z = quat.z / denom
    #     return x, y, z
