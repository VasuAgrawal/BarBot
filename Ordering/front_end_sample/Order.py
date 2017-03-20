class Order(object):

    def __init__(self, id, userId, drinkId, drinkName, time, robot=None):
        self.id = id
        self.userId = userId
        self.drinkType = drinkName
        self.drinkId = drinkId
        self.completed = False
        self.time = time
        self.timeStr = str(time)

        self.robot = robot