import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy as rp

def lidarCallback(data):
    lidarData = data.ranges
    driveDirection = rightDrive(findRightSmallest(lidarData))
    pub.publish(driveDirection+",1")

def findRightSmallest(arry):
    rightArray = arry[0:541]
    smallestRight = min(rightArray)
    smallestRightAngle = (arry.index(smallestRight)+1)/4
    return [smallestRight, smallestRightAngle]

def findLeftSmallest(arry):
    leftArray = arry[542:1080]
    smallestLeft = min(leftArray)
    print(smallestLeft)
    smallestLeftAngle = (arry.index(smallestLeft)+542)/4
    return [smallestLeft, smallestLeftAngle]


def rightDrive(rightSmallest):
    steer = ((rightSmallest[0] * math.sin(math.radians(90 - rightSmallest[1]))) - 0.61) * .34
    speed = (rightSmallest[0] * math.cos(math.radians(90 - rightSmallest[1])))
    return str(.5) + "," + str(steer)

rp.init_node("serpentine",anonymous=False)
rp.Subscriber("scan",LaserScan,lidarCallback)
pub = rp.Publisher("serpentine",String,queue_size=10)
rp.spin()
