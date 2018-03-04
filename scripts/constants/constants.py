#!/usr/bin/python
from helpers import Helpers

class Constants():

    def __init__(self, filename):
        self.constants = {}
        self.filename = filename
        self.loadFile()

    def get(self, key):
        return self.constants[key]

    def getAll(self):
        return self.constants

    def set(self, key, value):
        self.constants[key] = value
        self.saveFile()

    def loadFile(self):
        with open(self.filename) as lines:
            for line in lines:
                line = Helpers.removeNewline(line)
                key, value = line.split(': ')
                self.set(key, value)

    def saveFile(self):
        with open(self.filename, 'w') as output:
            for key, value in self.constants.items():
                output.write(str(key) + ': ' + str(value) + '\n')
