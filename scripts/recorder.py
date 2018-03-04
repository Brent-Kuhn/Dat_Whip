#!/usr/bin/python
import rospy as rp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

OUTPUT_FILENAME = 'output/output.avi'
FPS = 60
IMAGE_WIDTH = 376
IMAGE_HEIGHT = 672

class Recorder():
    def __init__(self):
        self.bridge = CvBridge()

    def recordChannel(self, channel):
        self.output = cv2.VideoWriter(OUTPUT_FILENAME, Recorder.getCodec(), \
            FPS, IMAGE_WIDTH, IMAGE_HEIGHT)
        rp.Subscriber(channel, Image, self.recordFrame)
        rp.spin()

    @staticmethod
    def getCodec():
        return cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

    def recordFrame(self, message):
        image = self.getCVImageFromMessage(message)
        self.output.write(image)

    def getCVImageFromData(self, data):
        return self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def __del__(self):
        self.output.close()

if __name__ == '__main__':
    try:
        Recorder().recordChannel('debugLeft')
    except rp.ROSInterruptException:
        pass
