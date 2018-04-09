import cv2
import numpy as np
import math
import os

def findCenter(mask):
    blur = cv2.GaussianBlur(mask,(5,5),0)
    contours = cv2.findContours(blur,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    M = cv2.moments(contours[0])
    if(M["m00"]==0):
        return 0,0
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX,cY

def findObject(hsv):
    orangeMin = np.array([0,100,100])
    orangeMax = np.array([10,255,255])
    maskOrange = cv2.inRange(hsv,orangeMin,orangeMax)

    minBlue = np.array([100,75,75])
    maxBlue = np.array([120,255,255])
    maskBlue = cv2.inRange(hsv,minBlue,maxBlue)
    mask=cv2.bitwise_or(maskBlue,maskOrange)
    return mask

def video():
    cap = cv2.VideoCapture(0)
    while(cap.isOpened()):
        ret,image = cap.read()
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask = findObject(hsv)
        x,y = findCenter(mask)
        cv2.circle(image, (x, y), 7, (0, 0, 255), -1)
        cv2.imshow("Images",image)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
def steer(height,width,x,y):
    y = height - y
    angle = -.34 * math.atan2(x, y) *(2/math.pi)
    speed = y / (height/2)
    return(str(speed)+","+str(angle)+","+"0")

def picture():
    # image = cv2.imread("test.jpg")
    # image = cv2.imread("pictures/block48.jpg")
    image = cv2.imread("coneOut/capL5.png")
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask = findObject(hsv)
    x,y = findCenter(mask)
    height, width, _ = image.shape
    print(steer(height,width,x,y))

    cv2.circle(image, (x, y), 7, (0, 0, 255), -1)
    cv2.imshow("Images",image)
    cv2.waitKey(0)

picture()
