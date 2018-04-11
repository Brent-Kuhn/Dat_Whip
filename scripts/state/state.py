class State(object):

    def error(self, data):
        return 0

    def shouldChangeState(self, data):
        return False

    def nextState(self, data):
        return State()
