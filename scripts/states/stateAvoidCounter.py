from states.stateAvoid import StateAvoid

class StateAvoidCounter(StateAvoid):

    def getMinAngle(self):
        return -StateAvoid.getMinAngle(self)

    def getMaxAngle(self):
        return -StateAvoid.getMaxAngle(self)

    def getGoalAngle(self):
        return -StateAvoid.getGoalAngle(self)

    def getSteerDirection(self):
        return -StateAvoid.getSteerDirection(self)
