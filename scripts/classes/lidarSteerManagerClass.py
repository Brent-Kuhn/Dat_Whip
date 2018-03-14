#!/usr/bin/python

from PID import PID
import time
from constants import LIDAR_STEER_SPEED, LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D

class LidarSteerManager:
    def __init__(self):
        self.pid = PID(PID_P, PID_I, PID_D)
        self.pid.SetPoint = 0
        self.pid.setSampleTime(0.01)

    def steer(self, leftDistance, rightDistance):
        error = rightDistance - leftDistance
        self.pid.update(error)
        return LIDAR_STEER_SPEED * self.pid.output

def main():
    steerManager = LidarSteerManager()
    with open('testData.txt') as file:
        for line in file:
            line = line[:-1] # remove newline at end
            data = line.split(',')
            leftDistance = float(data[0])
            rightDistance = float(data[1])
            steer = steerManager.steer(leftDistance, rightDistance)
            print('%3.1f -- %3.1f : %3.2f.' % (leftDistance, rightDistance, steer))
            time.sleep(0.1)

if __name__ == '__main__':
    main()
