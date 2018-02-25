#!/usr/bin/python
import numpy as np

class LineHistoryClass:

    def __init__(self, storage_size):
        self.storage_size = storage_size
        self.last_line = [0] * 4
        self.__initStorage()
        self.store_index = 0

    def __initStorage(self):
        self.storage = []
        for i in range(self.storage_size):
            self.storage.append([0] * 4)

    def getAverage(self):
        moving_avg_line = [0] * 4
        for i in range(4):
            for line in self.storage:
                moving_avg_line[i] += line[i]
            moving_avg_line[i] /= self.storage_size
        return moving_avg_line

    def addToHistory(self, line):
        if np.sum(self.storage) == 0:
            self.__fillHistoryWith(line)
        for i in range(4):
            self.storage[self.store_index % self.storage_size][i] = line[i]
        self.store_index += 1

    def getLastLine(self):
        last_index = self.store_index - 1
        return self.storage[last_index % self.storage_size]

    def __fillHistoryWith(self, line):
        for i in range(self.storage_size):
            for j in range(4):
                self.storage[i][j] = line[j]
