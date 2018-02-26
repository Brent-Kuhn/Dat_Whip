#!/usr/bin/python
import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from classes.lineFinderClass import LineFinder

class LineFinderSubscriber:
    def __init__(self):
        rp.init_node('image_processor', anonymous=False)
        self.pub = rp.Publisher('lineCoords', String, queue_size=10)
        self.lastLeftGoal = (0, 0)
        self.lastRightGoal = (0, 0)
        self.lineFinder = LineFinder()
        self.bridge = CvBridge()
        self.subscribeToBoth()

    def subscribeToBoth(self):
        self.subscribeToLeft()
        self.subscribeToRight()
        rp.spin()

    def subscribeToLeft(self):
        self.subscribeTo('zedLeft', self.zedLeftCallback)

    def subscribeToRight(self):
        self.subscribeTo('zedRight', self.zedRightCallback)

    def subscribeTo(self, channel, callback):
        rp.Subscriber(channel, Image, callback)

    def zedLeftCallback(self, data):
        image = self.getCVImageFromData(data)
        line = self.lineFinder.findIn(image)
        height, width, _ = image.shape
        goalX, goalY = self.getGoalXYFromLine(line, width, height)
        # self.publishXY(goalX, goalY)
        self.lastLeftGoal = (goalX, goalY)

        # Actually publish stuff
        goalX, goalY = self.averageGoal()
        self.publishXY(goalX, goalY)

    def zedRightCallback(self, data):
        image = self.getCVImageFromData(data)
        line = self.lineFinder.findIn(image)
        height, width, _ = image.shape
        goalX, goalY = self.getGoalXYFromLine(line, width, height)
        # self.publishXY(goalX, goalY)
        self.lastRightGoal = (goalX, goalY)

    def averageGoal(self):
        leftX = self.lastLeftGoal[0]
        leftY = self.lastLeftGoal[1]
        rightX = self.lastRightGoal[0]
        rightY = self.lastRightGoal[1]
        avgX = (leftX + rightX) / 2
        avgY = (leftY + rightY) / 2
        return avgX, avgY

    def getCVImageFromData(self, data):
        return self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    @classmethod
    def getGoalXYFromLine(self, line, width, height):
        try:
            [topX, topY, _, _] = line
            x = topX - int(width / 2)
            y = height - topY
            return x, y
        except Exception:
            return 0, 0

    def publishXY(self, x, y):
        self.pub.publish(str(x) + ',' + str(y))

if __name__ == '__main__':
    try:
        LineFinderSubscriber()
    except rp.ROSInterruptException:
        pass
