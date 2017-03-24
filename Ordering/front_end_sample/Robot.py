class Robot(object):
    orderCapacity = 2

    def __init__(self, id, location=None):
        self.id = id
        self.location = location
        self.orders = [None] * Robot.orderCapacity
        self.inTransit = False

    def getLocation(self):
        return self.location

    def getID(self):
        return self.id

    def updateLocation(self, newLocation):
        self.location = newLocation

    def assignOrder(self, order):
        if self.inTransit: return False #can't add more drinks while not at the bar
        for i in range(len(self.orders)):
            if self.orders[i] == None:
                self.orders[i] = order
                return True
        return False

    def getOrders(self):
        return list(filter(lambda x: x != None, self.orders))

    def removeOrder(self, order):
        assert(order in self.orders)
        shift = 0
        for i in range(len(self.orders)-1):
            if self.orders[i] == order:
                shift = 1
            self.orders[i] = self.orders[i+shift]
        self.orders[-1] = None