#!/usr/bin/python

from PID import PID
import time
from constants import LIDAR_STEER_SPEED, LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D

class LidarSteerManager:
    def __init__(self):
        self.pid = PID(LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D)
        self.pid.SetPoint = 0
        self.pid.setSampleTime(0.01)

    def steer(self, lidarData):
        error = self.error(lidarData)
        self.pid.update(error)
        return LIDAR_STEER_SPEED * error#self.pid.output

    def error(self, lidarData):
        leftArray, rightArray = lidarData
        error = sum(rightArray) - sum(leftArray)
        return error

def main():
    steerManager = LidarSteerManager()
    with open('testData.txt') as file:
        for line in file:
            line = line[:-1] # remove newline at end
            data = line.split(',')
            leftDistances = [float(data[0])]
            rightDistances = [float(data[1])]
            steer = steerManager.steer([leftDistances, rightDistances])
            print('%3.1f -- %3.1f : %3.2f.' % (leftDistances[0], rightDistances[0], steer))
            time.sleep(0.1)

if __name__ == '__main__':
    main()
