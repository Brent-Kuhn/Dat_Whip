#!/usr/bin/python
import unittest
from constants import Constants
from helpers import Helpers
import os

class TestConstants(unittest.TestCase):

    def setUp(self):
        self.TEST_FILENAME = 'test.txt'
        self.createTestFile()
        self.constants = Constants(self.TEST_FILENAME)

    def tearDown(self):
        self.removeTestFile()

    def testLoadFile(self):
        assert self.constants.constants['A'] == 'a'
        assert self.constants.constants['B'] == 'b'
        assert self.constants.constants['C'] == 'c'

    def testGet(self):
        assert self.constants.get('A') == 'a'
        assert self.constants.get('B') == 'b'
        assert self.constants.get('C') == 'c'

    def testSet(self):
        self.constants.set('C', 'foo')
        assert self.constants.get('A') == 'a'
        assert self.constants.get('B') == 'b'
        assert self.constants.get('C') == 'foo'

    def testSaveFile(self):
        self.constants.set('C', 'bar')
        constants2 = Constants(self.TEST_FILENAME)
        assert constants2.get('A') == 'a'
        assert constants2.get('B') == 'b'
        assert constants2.get('C') == 'bar'

    def testGetAll(self):
        c = self.constants.getAll()
        assert c['A'] == 'a'
        assert c['B'] == 'b'
        assert c['C'] == 'c'
        assert len(c.keys()) == 3

    def createTestFile(self):
        try:
            with open(self.TEST_FILENAME, 'w') as testFile:
                testFile.write('A: a\n')
                testFile.write('B: b\n')
                testFile.write('C: c\n')
        except IOError:
            pass

    def removeTestFile(self):
        Helpers.removeIfExists(self.TEST_FILENAME)

if __name__ == '__main__':
    unittest.main()
