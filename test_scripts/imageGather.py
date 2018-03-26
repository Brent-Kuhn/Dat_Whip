import cv2
import numpy as np


cap = cv2.VideoCapture(0)

# Read until video is completed
i=0
while(cap.isOpened()):
    # Capture frame-by-frame
    ret, img = cap.read()
    cv2.imwrite("cap"+i+".png",img)
    i+=1
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# When everything done, release the video capture object
cap.release()

# Closes all the frames
