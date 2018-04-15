from states.stateCircleRight import StateCircleRight

class StateCircleLeft(StateCircleRight):

    def nextState(self, lidar, zed, imu):
        return 'StateGoToLeft'

    def getMinAngle(self):
        return -StateCircleRight.getMinAngle(self)

    def getMaxAngle(self):
        return -StateCircleRight.getMaxAngle(self)

    def getGoalAngle(self):
        return -StateCircleRight.getGoalAngle(self)

    def getSteerDirection(self):
        return -StateCircleRight.getSteerDirection(self)
