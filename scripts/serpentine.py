#!/usr/bin/python
import rospy as rp
from classes.LidarHelperClass import LidarHelper
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from classes.PID import PID
from states.stateCircleLeft import StateCircleLeft
from states.stateCircleRight import StateCircleRight
from states.stateAvoidCounter import StateAvoidCounter
from states.stateTurnLeft import StateTurnLeft
from states.stateTurnRight import StateTurnRight
from states.stateGoToLeft import StateGoToLeft
from states.stateGoToRight import StateGoToRight
from states.state45Left import State45Left
from states.state45Right import State45Right
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

        self.lastState = ''

        self.imu = None

        self.lastRecordedTime = 0
        self.zedImage = None
        self.capture = cv2.VideoCapture(1)
        self.subscribeToImu()
        self.subscribeToLidar()
        rp.spin()

    def subscribeToImu(self):
        rp.Subscriber('imu', Imu, self.imuCallback)

    def imuCallback(self, data):
        self.imu = data

    def subscribeToLidar(self):
        rp.Subscriber('scan', LaserScan, self.callback)

    def callback(self, data):
        self.readZedImage()
        error = self.state.error(data.ranges, self.zedImage, self.imu)
        curState = type(self.state).__name__
        if curState != self.lastState:
            self.lastState = curState
            print(curState)
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
        if self.state.shouldChangeState(data.ranges, self.zedImage, self.imu):
            stateName = self.state.nextState(data.ranges, self.zedImage, self.imu)
            self.state = Serpentine.stateFromStateName(stateName)

    @staticmethod
    def stateFromStateName(stateName):
        return {
            'StateGoToRight': StateGoToRight(),
            'StateGoToLeft': StateGoToLeft(),
            'StateTurnRight': StateTurnRight(),
            'StateTurnLeft': StateTurnLeft(),
            'StateCircleLeft': StateCircleLeft(),
            'StateCircleRight': StateCircleRight(),
            'State45Left': State45Left(),
            'State45Right': State45Right()
        }[stateName]

if __name__ == '__main__':
    try:
        Serpentine()
    except rp.ROSInterruptException:
        pass
