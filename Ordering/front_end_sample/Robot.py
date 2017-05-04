import math

def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class Robot(object):

    def __init__(self, id, location=None, capacity=2):
        self.id = id
        self.capacity = capacity
        self.orders = []
        self.inTransit = False
        self.goal = None
        self.goalTime = None
        self.waitTime = 4 # 10 seconds

    def getLocation(self, locations):
        locationMap = locations.locations

        if self.id in locationMap:
            position = locationMap[self.id]
            # do we need to use z here at all?
            return (position.x, position.y, position.z)
        else:
            return (0,0,0)
            #raise Exception("Cannot find wristband id %d in location map %r" % (self.id, locationMap))

    def getID(self):
        return self.id

    def getOrders(self):
        return self.orders

    def getTripDistance(self, barX, barY, locations):
        if not self.inTransit:
            return 0
        else:
            # currently on a trip
            (currX, currY, currZ) = self.getLocation(locations)
            totalDist = 0
            for order in self.getOrders():
                (orderX, orderY, orderZ) = order.getLocation(locations)
                totalDist += distance(currX, currY, orderX, orderY)
                (currX, currY) = (orderX, orderY)
            totalDist += distance(currX, currY, barX, barY)
            return totalDist

    def isReady(self):
        orders = self.getOrders()
        if len(orders) > 0:
            for order in orders:
                if not order.completed:
                    return False
            self.inTransit = True
            return True
        return False

