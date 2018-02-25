#!/usr/bin/python

class estop:
    def __init__(self,attempts,start,stop):
        self.maxAttempts = attempts
        self.attempts = 0
        self.start = start
        self.stop = stop

    def processInput(self,ranges):
        ranges=ranges[self.start:self.stop]
        if min(ranges)<.1778 and self.attempts<self.maxAttempts:
            self.attempts+=1
            return(-1,0)
        elif min(ranges)>.1778:
            self.attempts=0
