from states.stateAvoid import StateAvoid
from states.stateGoToLeft import StateGoToLeft

class StateAvoidCounter(StateAvoid):

    def nextState(self, data):
        return StateGoToLeft()

    def getMinAngle(self):
        return -StateAvoid.getMinAngle(self)

    def getMaxAngle(self):
        return -StateAvoid.getMaxAngle(self)

    def getGoalAngle(self):
        return -StateAvoid.getGoalAngle(self)

    def getSteerDirection(self):
        return -StateAvoid.getSteerDirection(self)
