#!/usr/bin/python
import math

from PID import PID
import time
from classes.LidarHelperClass import LidarHelper
from constants import MAX_STEER, LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D, \
    LIDAR_PID_SAMPLE_RATE, LIDAR_STEER_MULT

class LidarSteerManager:
    def __init__(self):
        self.pid = PID(LIDAR_PID_P, LIDAR_PID_I, LIDAR_PID_D)
        self.pid.SetPoint = 0
        self.pid.setSampleTime(LIDAR_PID_SAMPLE_RATE)
        self.priority = "0"

    def steer(self, lidarData):
        error = self.error(lidarData)
        self.pid.update(error)
        speed = .5 + .5 * (1 - abs(self.pid.output))
        return MAX_STEER * self.pid.output,speed,self.priority

    def error(self, points):
        FUTURE_WEIGHT = .65
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
        # Attempting to code without car.....
        if(left < .5 or right < .5):
            self.priority = "2"
        else:
            self.priority = "0"
        sub_error = right - left
        return sub_error

    def getPerpDistance(self, lidarPoints, angle):
        # Perpendicular
        index = LidarHelper.angleToLidarIndex(angle)
        vectorDistance = lidarPoints[index]
        return abs(LidarSteerManager.getXComponentOfVector(vectorDistance, 90 - angle))

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
