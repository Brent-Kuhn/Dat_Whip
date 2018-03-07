#!/usr/bin/python
import cv2
import rospy as rp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

OUTPUT_FILENAME = 'output/output.avi'
FPS = 30
IMAGE_HEIGHT = 376 * 2
IMAGE_WIDTH = 672 * 3

class Recorder():
    def __init__(self):
        self.bridge = CvBridge()
        self.output = cv2.VideoWriter(OUTPUT_FILENAME, Recorder.getCodec(), \
            FPS, (IMAGE_WIDTH, IMAGE_HEIGHT))

    def recordChannel(self, channel):
       rp.init_node('recorder', anonymous=True)
       rp.Subscriber(channel, Image, self.recordFrame)
       rp.spin()

    @staticmethod
    def getCodec():
        return cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

    def recordFrame(self, message):
        image = self.getCVImageFromMessage(message)
        self.output.write(image)

    def getCVImageFromMessage(self, data):
        return self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def __del__(self):
        self.output.release()

if __name__ == '__main__':
    try:
        Recorder().recordChannel('debugLeft')
    except rp.ROSInterruptException:
        pass
