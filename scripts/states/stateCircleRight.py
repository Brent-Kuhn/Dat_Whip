from classes.LidarHelperClass import LidarHelper
from object_detection.object_detection_images import findMainObject
import math

class StateCircleRight(object):

    def __init__(self):
        self.startOrientationWasSet = False

    def shouldChangeState(self, lidar, zed, imu):
        # return self.hasTurned180(imu) and self.somethingIsInFront(lidar, zed)
        return self.somethingIsInFront(lidar, zed)

    def hasTurned180(self, imu):
        if not self.startOrientationWasSet:
            self.startOrientationWasSet = True
            # self.startOrientation = self.verticalOrientation(imu)
            self.startOrientation = time()
        # offsetOrient = self.verticalOrientation(imu) - self.startOrientation
        # print(offsetOrient)
        # return abs(offsetOrient) > .5
        return time() > self.startOrientation + 5 # 5 seconds

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

    def somethingIsInFront(self, lidar, zed):
        if not self.lidarSomethingIsInFront(lidar):
            return False
        self.zedObject = findMainObject(zed)
        return self.zedSomethingIsInFront()

    def lidarSomethingIsInFront(self, lidar):
        FRONT_RANGE = 2
        _, minDistance = LidarHelper.shortestDistInRange(lidar, -FRONT_RANGE, FRONT_RANGE)
        return minDistance < 2.3

    def zedSomethingIsInFront(self):
        return self.zedObject != ''

    def zedConeIsInFront(self):
        return self.zedObject == 'cone'


    def nextState(self, lidar, zed, imu):
        return 'StateGoToRight'

    def getMinAngle(self):
        return 90 - 45

    def getMaxAngle(self):
        return 90 + 45

    def getGoalAngle(self):
        return 90

    def getGoalDistance(self):
        return .5

    def getSteerDirection(self):
        return -1

    def error(self, lidar, zed, imu):
        a = .5
        b = 1 - a
        minIndex, minDistance = LidarHelper.shortestDistInRange(lidar, self.getMinAngle(), self.getMaxAngle())
        # print('minIndex = %d, minDistance = %f' % (minIndex, minDistance))
        return a * self.errorAngle(minIndex) + b * self.errorDist(minDistance)

    def errorAngle(self, minIndex):
        minAngle = LidarHelper.lidarIndexToAngle(minIndex)
        # print('minAngle = %f' % minAngle)
        return (self.getGoalAngle() - minAngle) / abs(self.getMaxAngle() - self.getMinAngle())

    def errorDist(self, minDistance):
        return self.getSteerDirection() * (minDistance - self.getGoalDistance()) / self.getGoalDistance()
