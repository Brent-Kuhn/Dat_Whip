import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy as rp

def lidarCallback(data):
    lidarData = data.ranges
    smallestRight = findRightSmallest(lidarData)
    if smallestRight[0] < 1:
        driveDirection = rightDrive(smallestRight)
        pub.publish(driveDirection+",1")

def findRightSmallest(arry):
    rightArray = arry[10:541]
    smallestRight = min(rightArray)
    smallestRightAngle = (arry.index(smallestRight)+1)/4
    return [smallestRight, smallestRightAngle]

def findLeftSmallest(arry):
    leftArray = arry[542:1070]
    smallestLeft = min(leftArray)
    print(smallestLeft)
    smallestLeftAngle = (arry.index(smallestLeft)+542)/4
    return [smallestLeft, smallestLeftAngle]


def rightDrive(rightSmallest):
    steer = (.46 - (rightSmallest[0] * math.cos(math.radians(45 - rightSmallest[1]))))
    speed = (rightSmallest[0] * math.sin(math.radians(45 - rightSmallest[1])))
    return str(1) + "," + str(steer)

rp.init_node("serpentine",anonymous=False)
rp.Subscriber("scan",LaserScan,lidarCallback)
pub = rp.Publisher("serpentine",String,queue_size=10)
rp.spin()
