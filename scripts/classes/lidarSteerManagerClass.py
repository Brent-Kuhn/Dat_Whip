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
        speed = 1 - abs(self.pid.output)
        return MAX_STEER * self.pid.output,speed

    def error(self, points):
        FUTURE_WEIGHT = .25
        PRESENT_WEIGHT = 1 - FUTURE_WEIGHT
        diff_present = self.subErrorAtAngle(points, 90 - 5)
        diff_future = self.subErrorAtAngle(points, 45)
        diff = diff_present * PRESENT_WEIGHT + diff_future * FUTURE_WEIGHT
        return LidarSteerManager.cappedMultiply(diff, LIDAR_STEER_MULT)

    def subErrorAtAngle(self, lidarPoints, angle):
        # angle = degrees
        left = self.getPerpDistance(lidarPoints, -angle)
        right = self.getPerpDistance(lidarPoints, angle)
        left, right = LidarSteerManager.normalize(left, right)
        sub_error = right - left
        return sub_error

    def getPerpDistance(self, lidarPoints, angle):
        # Perpendicular
        index = self.angleToLidarIndex(angle)
        vectorDistance = lidarPoints[index]
        return abs(LidarSteerManager.getXComponentOfVector(vectorDistance, 90 - angle))

    def angleToLidarIndex(self, angle):
        # angle = degrees
        # 0 = center = 540
        # - = left, + = right
        MAX_ANGLE = 135
        angle = LidarSteerManager.clip(int(angle), MAX_ANGLE)
        return 1080 / 2 - 4 * angle

    @staticmethod
    def clip(value, absMaxValue):
        value = max(value, -absMaxValue)
        value = min(value, absMaxValue)
        return value

    @staticmethod
    def getXComponentOfVector(magnitude, direction):
        # direction = degree
        degree2Radian = math.pi / 180
        return magnitude * math.cos(direction * degree2Radian)

    @staticmethod
    def normalize(left_distance, right_distance):
        max_distance = max(left_distance, right_distance)
        return left_distance / max_distance, right_distance / max_distance

    @staticmethod
    def cappedMultiply(value, by):
        if value == 0:
            return 0
        return min(abs(value) * by, 1) * value / abs(value)

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
