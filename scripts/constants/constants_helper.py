#!/usr/bin/python
from helpers import Helpers
import rospy as rp
from std_msgs.msg import String

CONSTANTS_FILENAME = 'constants.yaml'

constants = {}

def getConstant(key):
    try:
        return constants[key]
    except:
        raise Exception('\'%s\' does not contain key: %s' % (CONSTANTS_FILENAME, key))

def loadConstants():
    with open(CONSTANTS_FILENAME) as constantsFile:
        for line in constantsFile:
            key, value = parse(line)
            constants[key] = value

def loadConstants(_):
    loadConstants()

def parse(line):
    try:
        line = Helpers.removeNewline(line)
        return line.split(': ')
    except:
        pass # invalid entry

if __name__ == '__main__':
    try:
        loadConstants()
        rp.init_node('constants', anonymous=False)
        rp.Subscriber('constantUpdate', String, loadConstants)
        rp.spin()
    except rp.ROSInterruptException:
        pass
