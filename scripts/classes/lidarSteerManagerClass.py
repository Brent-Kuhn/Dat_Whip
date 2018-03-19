#!/usr/bin/python
import math

from PID import PID
import time
from constants import MAX_STEER, LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D, \
    LIDAR_PID_SAMPLE_RATE, LIDAR_STEER_MULT

class LidarSteerManager:
    def __init__(self):
        self.pid = PID(LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D)
        self.pid.SetPoint = 0
        self.pid.setSampleTime(LIDAR_PID_SAMPLE_RATE)

    def steer(self, lidarData):
        error = self.error(lidarData)
        self.pid.update(error)
        return MAX_STEER * self.pid.output

    def error(self, points):
        right1 = points[180 - 1]
        right2 = points[240 - 1]
        left1 = points[900]
        left2 = points[840]

        right_perp = right2/(math.acos(math.pi/6))

        left_perp = left2/(math.acos(math.pi/6))

        right_avg = (right1 + right_perp)/2
        left_avg = (left1 + left_perp)/2

        max_avg = max(left_avg, right_avg)

        diff = (right_avg/max_avg - left_avg/max_avg)

        if diff == 0:
            return 0
        return min(abs(diff) * LIDAR_STEER_MULT, 1) * (diff / abs(diff))

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
