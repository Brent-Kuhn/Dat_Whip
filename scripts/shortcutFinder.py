#!/usr/bin/python
import rospy as rp
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from classes.LidarHelperClass import LidarHelper

IGNORE_RANGE = 5
SEARCH_SIZE = 3
DISTANCE_THRESH = 1

class ShortcutFinderSubscriber():
    def __init__(self):
        rp.init_node('shortcutFinder', anonymous=True)
        self.subscribeToLidar()
        rp.spin()

    def subscribeToLidar(self):
        rp.Subscriber('scan', LaserScan, self.lidarCallback)

    def lidarCallback(self, lidar):
        data = lidar.ranges
        # print(['%.2f' % x for x in data[0:6]])
        minIndex, minDistance = LidarHelper.shortestDistInRange(data, -20, 20)
        minAngle = LidarHelper.lidarIndexToAngle(minIndex)
        leftIndex, leftDistance = LidarHelper.shortestDistInRange(data, minAngle - IGNORE_RANGE - SEARCH_SIZE, minAngle + IGNORE_RANGE)
        rightIndex, rightDistance = LidarHelper.shortestDistInRange(data, minAngle + IGNORE_RANGE + SEARCH_SIZE, minAngle - IGNORE_RANGE)
        if leftDistance > DISTANCE_THRESH and rightDistance > DISTANCE_THRESH:
            print('no speakable angle')
            return # Neither is the wall. Keep searching.
        elif leftDistance < rightDistance:
            goalIndex = leftIndex
        else:
            goalIndex = rightIndex
        goalAngle = LidarHelper.lidarIndexToAngle(goalIndex)
        print('goalAngle = %f' % goalAngle)

if __name__ == '__main__':
    try:
        ShortcutFinderSubscriber()
    except rp.ROSInterruptException:
        pass
