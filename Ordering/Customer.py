class Customer(object):

    def __init__(self, id):
        self.id = id

    def getID(self):
        return self.id

    def getPosition(self):
        #use id to get wristband data
        raise Exception("Not yet implemented")

    def __repr__(self):
        return "Customer %d" % self.id