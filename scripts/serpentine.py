#!/usr/bin/python
import rospy as rp
from classes.LidarHelperClass import LidarHelper
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from classes.PID import PID
from states.state import State
from states.stateAvoid import StateAvoid
from states.stateAvoidCounter import StateAvoidCounter
from states.stateGoToRight import StateGoToRight
from states.stateGoToLeft import StateGoToLeft
import cv2
from time import time

SERPENTINE_PID_P = 1
SERPENTINE_PID_I = 0
SERPENTINE_PID_D = 0
PID_SAMPLE_RATE = 0.01

IMAGE_RATE = 1 / 30.0

class Serpentine():
    def __init__(self):
        self.pid = PID(SERPENTINE_PID_P, SERPENTINE_PID_I, SERPENTINE_PID_D)
        self.pid.SetPoint = 0
        self.pid.setSampleTime(PID_SAMPLE_RATE)

        rp.init_node('Serpentine', anonymous=True)
        self.state = StateGoToRight()
        self.pub = rp.Publisher('/serpentine', String, queue_size=10)

        self.lastRecordedTime = 0
        self.zedImage = None
        self.capture = cv2.VideoCapture(1)
        self.subscribeToLidar()
        rp.spin()

    def subscribeToLidar(self):
        rp.Subscriber('scan', LaserScan, self.callback)

    def callback(self, data):
        # print('\n\n')
        zedImage = self.readZedImage()
        error = self.state.error(data.ranges, zedImage)
        print(type(self.state).__name__)
        print('error = %f' % error)
        self.steer(error)
        self.updateState(data)

    def readZedImage(self):
        curTime = time()
        if curTime > self.lastRecordedTime + IMAGE_RATE:
            self.lastRecordedTime = curTime
            _, image = self.capture.read()
            self.zedImage = image
        return self.zedImage

    def steer(self, direction):
        self.pid.update(direction)

        # print('steer = %f, post-pid = %f' % (direction, -self.pid.output))
        self.pub.publish('1,' + str(-self.pid.output) + ',1')

    def updateState(self, data):
        if self.state.shouldChangeState(data.ranges, self.zedImage):
            stateName = self.state.nextState(data.ranges, self.zedImage)
            self.state = Serpentine.stateFromStateName(stateName)

    @staticmethod
    def stateFromStateName(stateName):
        return {
            'StateAvoid': StateAvoid(),
            'StateAvoidCounter': StateAvoidCounter(),
            'StateGoToRight': StateGoToRight(),
            'StateGoToLeft': StateGoToLeft()
        }[stateName]

if __name__ == '__main__':
    try:
        Serpentine()
    except rp.ROSInterruptException:
        pass
