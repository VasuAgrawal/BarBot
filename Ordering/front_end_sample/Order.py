class Order(object):

    def __init__(self, id, userId, drinkId, drinkName, time, robot=None, location=None):
        self.id = id
        self.userId = userId
        self.drinkType = drinkName
        self.drinkId = drinkId
        self.completed = False
        self.time = time
        self.timeStr = str(time)

        self.robot = robot
        self.location = location

    def assignRobot(self, robot):
        self.robot = robot

    def updateLocation(self, location):
        self.location = location

    def getLocation(self):
        return self.location

    def __eq__(self, other):
        return isinstance(other, Order) and self.id == other.id