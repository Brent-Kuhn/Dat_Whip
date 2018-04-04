#!/usr/bin/python
import rospy as rp
from classes.LidarHelperClass import LidarHelper
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from classes.PID import PID

class State(object):

    def error(self, data):
        return 0

    def shouldChangeState(self, data):
        return False

    def nextState(self, data):
        return State()

class StateAvoid(State):

    def getMinAngle(self):
        return 90 - 45

    def getMaxAngle(self):
        return 90 + 45

    def getGoalAngle(self):
        return 90

    def getGoalDistance(self):
        return 1

    def getSteerDirection(self):
        return -1

    def error(self, data):
        a = .5
        b = 1 - a
        minIndex, minDistance = LidarHelper.shortestDistInRange(data, self.getMinAngle(), self.getMaxAngle())
        print('minIndex = %d, minDistance = %f' % (minIndex, minDistance))
        return a * self.errorAngle(minIndex) + b * self.errorDist(minDistance)

    def errorAngle(self, minIndex):
        minAngle = LidarHelper.lidarIndexToAngle(minIndex)
        print('minAngle = %f' % minAngle)
        return (self.getGoalAngle() - minAngle) / abs(self.getMaxAngle() - self.getMinAngle())

    def errorDist(self, minDistance):
        return self.getSteerDirection() * (minDistance - self.getGoalDistance()) / self.getGoalDistance()

class StateAvoidCounter(StateAvoid):

    def getMinAngle(self):
        return -StateAvoid.getMinAngle(self)

    def getMaxAngle(self):
        return -StateAvoid.getMaxAngle(self)

    def getGoalAngle(self):
        return -StateAvoid.getGoalAngle(self)

    def getSteerDirection(self):
        return -StateAvoid.getSteerDirection(self)

SERPENTINE_PID_P = 1
SERPENTINE_PID_I = 0
SERPENTINE_PID_D = 0
PID_SAMPLE_RATE = 0.01

class Serpentine():
    def __init__(self):
        self.pid = PID(SERPENTINE_PID_P, SERPENTINE_PID_I, SERPENTINE_PID_D)
        self.pid.SetPoint = 0
        self.pid.setSampleTime(PID_SAMPLE_RATE)

        rp.init_node('Serpentine', anonymous=True)
        self.state = StateAvoid()
        self.pub = rp.Publisher('/serpentine', String, queue_size=10)
        self.subscribeToLidar()
        rp.spin()

    def subscribeToLidar(self):
        rp.Subscriber('scan', LaserScan, self.callback)

    def callback(self, data):
        print('\n\n')
        error = self.state.error(data.ranges)
        print('error = %f' % error)
        self.steer(error)
        # self.updateState(data)

    # def updateState(self, data):
    #     if self.state.shouldChangeState(data):
    #         self.state = self.state.nextState(data)

    def steer(self, direction):
        self.pid.update(direction)

        print('steer = %f, post-pid = %f' % (direction, -self.pid.output))
        self.pub.publish('1,' + str(-self.pid.output) + ',1')

if __name__ == '__main__':
    try:
        Serpentine()
    except rp.ROSInterruptException:
        pass
