#!/usr/bin/python
class LidarHelper():

    def __init__(self):
        pass

    @staticmethod
    def shortestDistInRange(data, angle1, angle2):
        index1 = LidarHelper.angleToLidarIndex(angle1)
        index2 = LidarHelper.angleToLidarIndex(angle2)
        index1, index2 = min(index1, index2), max(index1, index2)
        minSubIndex, minSubDist = LidarHelper.minWithIndex(data[index1:index2])
        return minSubIndex + index1, minSubDist

    @staticmethod
    def minWithIndex(list):
        size = len(list)
        if size == 0:
            return -1, -1
        minIndex = 0
        for i in range(size):
            if list[i] < list[minIndex]:
                minIndex = i
        return minIndex, list[minIndex]

    @staticmethod
    def angleToLidarIndex(angle):
        # angle = degrees
        # 0 = center = 540
        # - = left, + = right
        MAX_ANGLE = 135
        angle = LidarHelper.clip(int(angle), MAX_ANGLE)
        return 1080 / 2 - 4 * angle

    @staticmethod
    def lidarIndexToAngle(lidarIndex):
        # angle = degrees
        # 0 = center = 540
        # - = left, + = right
        MAX_INDEX = 1080
        lidarIndex = LidarHelper.clip(float(lidarIndex), MAX_INDEX)
        return (1080 / 2 - lidarIndex) / 4

    @staticmethod
    def clip(value, absMaxValue):
        value = max(value, -absMaxValue)
        value = min(value, absMaxValue)
        return value
