#!/usr/bin/python
import math

class LineDrive:
    def __init__(self,attempts):
        self.attemptTime = attempts
        self.backCount=0

    def processLine(self,lineData):
        x,y=lineData.split(",")
        x=float(x)
        y=float(y)
        if x != 0 and y != 0:
            angle = (-.34) * math.atan2(x, y) *(2/math.pi) # Assuming pos is left and neg is right
            speed = y / 100  #change this value to the max length of a line
            self.backCount = 0
            return(speed,angle)
        elif self.backCount < self.attemptTime:
            self.backCount+=1
            return(-2,0)
