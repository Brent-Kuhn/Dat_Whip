#!/usr/bin/python
import os

class Helpers():

    @staticmethod
    def removeNewline(line):
        if line[-1] == '\n':
            line = line[:-1]
        return line

    @staticmethod
    def removeIfExists(filename):
        try:
            os.remove(filename)
        except OSError:
            pass
