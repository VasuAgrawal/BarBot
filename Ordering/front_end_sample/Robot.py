import math

def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class Robot(object):

    def __init__(self, id, location=None, capacity=2):
        self.id = id
        self.location = location
        self.capacity = capacity
        self.orders = []
        self.inTransit = False
        self.speed = 5

    def getLocation(self):
        return self.location

    def getID(self):
        return self.id

    def updateLocation(self, newLocation):
        self.location = newLocation

    def getOrders(self):
        return self.orders

    def getTripDistance(self, data):
        if not self.inTransit:
            return 0
        else:
            # currently on a trip
            (currX, currY) = self.getLocation()
            totalDist = 0
            for order in self.getOrders():
                (orderX, orderY) = order.getLocation(data)
                totalDist += distance(currX, currY, orderX, orderY)
                (currX, currY) = (orderX, orderY)
            totalDist += distance(currX, currY, data.barX, data.barY)
            return totalDist

    def move(self, destX, destY):
        (robotX, robotY) = self.getLocation()
        dx, dy = destX - robotX, destY - robotY
        angle = math.atan2(dy, dx)
        dx = math.cos(angle) * self.speed
        dy = math.sin(angle) * self.speed
        self.updateLocation((robotX + dx, robotY + dy))

    def isReady(self):
        orders = self.getOrders()
        if len(orders) > 0:
            for order in orders:
                if not order.completed:
                    return False
            self.inTransit = True
            return True
        return False

