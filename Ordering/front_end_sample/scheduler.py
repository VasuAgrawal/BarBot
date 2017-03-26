'''
Scheduler for the BarBot system

Needs the following functionality:
    - receive an order from the ordering system
    - receive position updates from the localization system
    - assign orders to robots
    - send order queue to the ordering system
'''

from Order import Order
from Robot import Robot

class Scheduler(object):

    def __init__(self, numRobots):
        self.robots = []
        for robotID in range(numRobots):
            self.robots.append(Robot(robotID, (0, 0)))
        self.queue = []

    def updateRobotLocation(self, robotID, newLocation):
        for robot in self.robots:
            if robot.getID() == robotID:
                robot.updateLocation(newLocation)
                break
        #probably also want to update the queue here

    def updateCustomerLocation(self, orderID, newLocation):
        for order in self.queue:
            if order.id == orderID:
                order.updateLocation(newLocation)
                break
        #probably also want to update the queue here/reassign robots

    def addOrder(self, order):
        self.queue.append(order)
        #will want to do something more intelligent here than just a normal queue
        for robot in self.robots:
            if robot.assignOrder(order):
                order.assignRobot(robot)
                return
            
    def removeOrder(self, order):
        if order in self.queue:
            self.queue.remove(order)
        for robot in self.robots:
            if order in robot.getOrders():
                robot.removeOrder(order)
                order.robot = None
                break

    def updateRobot(self, robot):
        for order in self.queue:
            if order.robot == None:
                if robot.assignOrder(order):
                    order.assignRobot(robot)

    def getOrders(self):
        return self.queue