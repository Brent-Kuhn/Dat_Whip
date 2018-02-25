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
        self.lineFinder = LineFinder
        self.bridge = CvBridge()
        self.subscribeToLeft()

    def subscribeToLeft(self):
        rp.Subscriber('zedLeft', Image, zedLeftCallback)
        rp.spin()

    def zedLeftCallback(data):
        image = getCVImageFromData(data)
        line = self.lineFinder.findIn(image)
        height, width, _ = image.shape
        goalX, goalY = getGoalXYFromLine(line, width, height)
        self.publishXY(goalX, goalY)

    @classmethod
    def getCVImageFromData(data):
        return self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    @classmethod
    def getGoalXYFromLine(line, width, height):
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
