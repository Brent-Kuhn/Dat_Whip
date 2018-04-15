import cv2
import numpy as np
import time

def selectRegion(image):
        orangeMin = np.array([0,100,100])
        orangeMax = np.array([10,255,255])
        blueMin = np.array([100,90,90])
        blueMax = np.array([10,255,255])

        img = image[100:300, 50:600]
        cv2.imshow('OG', image)
        cv2.imshow('test',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
        #image = cv2.imread('IMG_4731.JPG')
        capture = cv2.VideoCapture(1)
        for x in range(0,200):
                _, image = capture.read()
        cv2.imwrite('Output1.JPG',image)
        #selectRegion(image)


main()
