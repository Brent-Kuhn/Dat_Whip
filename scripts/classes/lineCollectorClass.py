#!/usr/bin/python
from constants import SLOPE_MIN_THRESH, HISTORY_SIZE
from lineHistoryClass import LineHistory

class LineCollector:
    def __init__():
        self.history = LineHistory(HISTORY_SIZE)

    def processLines(self, lines):
        if lines is None:
            return []
        else:
            lines = self.removeThatStupidDimension(lines)
            lines = self.filterBySlope(lines)
            lines = self.ensureLinesGoTopToBottom(lines)
            avg_line = self.averageLines(lines, self.history.getLastLine())
            self.history.addToHistory(avg_line)
            moving_left_avg = self.history.getAverage()
            return [moving_left_avg]

    def removeThatStupidDimension(self, lines):
        output = []
        for line in lines:
            output.append(line[0])
        return output

    def filterBySlope(self, lines):
        output = []
        for line in lines:
            m = self.slope(line)
            if abs(m) > SLOPE_MIN_THRESH:
                output.append(line)
        return output

    def ensureLinesGoTopToBottom(self, lines):
        size = len(lines)
        for i in range(size):
            [x1, y1, x2, y2] = lines[i]
            if y1 > y2:
                lines[i] = [x2, y2, x1, y1]
        return lines

    def averageLines(self, lines, default):
        if len(lines) == 0:
            return default
        avg_line = [0] * 4
        for line in lines:
            for i in range(4):
                avg_line[i] += line[i]
        for i in range(4):
            avg_line[i] /= len(lines)
        return avg_line

    def slope(self, line):
        x1, y1, x2, y2 = line
        if x1 == x2:
            return float('inf')
        return (float(y1) - float(y2)) / (float(x1) - float(x2))
