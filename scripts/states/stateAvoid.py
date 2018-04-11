from classes.LidarHelperClass import LidarHelper

class StateAvoid(object):

    def shouldChangeState(self, lidar, zed):
        # TODO IS CONE OR CUBE
        _, minDistance = LidarHelper.shortestDistInRange(lidar, -10, 10)
        return minDistance < .6

    def nextState(self, lidar, zed):
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

    def error(self, lidar, zed):
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
