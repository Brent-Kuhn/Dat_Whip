#!/usr/bin/python
import unittest
from helpers import Helpers

class TestHelpers(unittest.TestCase):

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def testRemoveNewLine(self):
        self.assertEqual(Helpers.removeNewline('foo'), 'foo')
        self.assertEqual(Helpers.removeNewline('foo\n'), 'foo')

    def testRemoveIfExistsWhenFileExists(self):
        with open('test.txt', 'w') as file:
            file.write('foobar')
        Helpers.removeIfExists('test.txt')

    def testRemoveIfExistsWhenFileDoesNotExist(self):
        Helpers.removeIfExists('test.txt') # make sure it doesn't exist
        Helpers.removeIfExists('test.txt')
