import random

class Order(object):

    def __init__(self, id, wristbandId=None, drinkType=None, completed=False, time=None, robot=None, priority=None):
        self.id = id
        self.wristbandId = wristbandId
        self.drinkType = drinkType
        self.completed = completed
        self.time = time
        self.robot = robot
        self.priority = priority

    def getLocation(self, locations):
        locationMap = locations.locations

        if self.wristbandId in locationMap:
            customerPoint = locationMap[self.wristbandId]
            # do we need to use z here at all?
            return (customerPoint.x, customerPoint.y)
        else:
            # TODO: what should we do if the wristband doesn't exist?
            return (0, 0)
            #raise Exception("Cannot find wristband id %d in location map %r" % (self.wristbandId, locationMap))

    def __eq__(self, other):
        return isinstance(other, Order) and self.id == other.id

    def __repr__(self):
        robotId = -1 if self.robot == None else self.robot.id
        return "Order(%r, %r, %r)" % (self.id, self.completed, robotId)

    def __hash__(self):
        return hash(self.userId)