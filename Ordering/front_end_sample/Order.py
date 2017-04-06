class Order(object):

    def __init__(self, id, userId, drinkId, drinkType=None, completed=False, time=None, robot=None, priority=None):
        self.id = id
        self.userId = userId
        self.drinkId = drinkId
        self.drinkType = drinkType
        self.completed = completed
        self.time = time
        self.robot = robot
        self.priority = priority

    def getLocation(self, data):
        return data.locations[self.id]

    def __eq__(self, other):
        return isinstance(other, Order) and self.id == other.id

    def __repr__(self):
        robotId = -1 if self.robot == None else self.robot.id
        return "Order(%r, %r, %r)" % (self.id, self.completed, robotId)

    def __hash__(self):
        return hash(self.userId)