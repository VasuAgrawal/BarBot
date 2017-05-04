#!/usr/bin/env python3
import logging

import copy
import datetime
import math
import momoko
from Order import Order
from Robot import Robot
import packet
import time

import tornado
import tornado.tcpserver
import tornado.gen
import tornado.ioloop
from tornado import httpclient

from positions_pb2 import Point
from positions_pb2 import Locations
from positions_pb2 import ConnectionRequest

# TODO: add locks and stuff for reads/writes to self._loc

def distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

class Scheduler(tornado.tcpserver.TCPServer):
    def __init__(self, ioloop, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.http_client = httpclient.AsyncHTTPClient()
        self.destination = 'http://localhost:8080/scheduler/'

        self.bartenderId = 10
        # TODO: pick a reasonable number here
        self.thresh = 1 # acceptable distance from customer for drop offs
        self.numRobots = 1
        self.robots = dict()
        for i in range(self.numRobots):
            # TODO: replace this with actual wristband id
            robotWristbandId = i + 7
            self.robots[robotWristbandId] = Robot(robotWristbandId, capacity=1)
        self.orderQueue = []

        self._real_robots = dict()
        self._loc = Locations()

    @tornado.gen.coroutine
    def handle_gls(self, stream, address):
        logging.info("Received GLS connection request!")

        while True:
            try:
                # Continually read data from the stream
                message_bytes = yield packet.read_packet_from_stream(stream)
            except tornado.iostream.StreamClosedError:
                return

            loc = Locations() #map of wristband id to location
            loc.ParseFromString(message_bytes)
            logging.info("Received location data!")
            self._loc = loc

    @tornado.gen.coroutine
    def handle_robot(self, robotId, stream, address):
        logging.info("Received ROBOT connection request!")
        self._real_robots[robotId] = stream

    @tornado.gen.coroutine
    def handle_stream(self, stream, address):
        logging.info("Incoming connection request from %s", address)

        try:
            message_bytes = yield packet.read_packet_from_stream(stream)
        except tornado.iostream.StreamClosedError:
            return

        req = ConnectionRequest()
        req.ParseFromString(message_bytes)
        logging.debug("Received connection request saying %s", req)

        if req.type == ConnectionRequest.GLS:
            self.handle_gls(stream, address)
        elif req.type == ConnectionRequest.ROBOT:
            self.handle_robot(req.robotId, stream, address)
        else:
            logging.error("Invalid connection request state?")

    def update_robots(self):
        for robotId in self.robots:
            robot = self.robots[robotId]
            goal = robot.goal
            location = copy.copy(self._loc)
            if goal == None: # without a goal should stay in place
                (location.waypoint.x, location.waypoint.y, 
                    location.waypoint.z) = robot.getLocation(self._loc)
            else:
                (location.waypoint.x, location.waypoint.y, 
                    location.waypoint.z) = goal
            data = packet.make_packet_from_bytes(location.SerializeToString())
            if robotId in self._real_robots:
                realRobot = self._real_robots[robotId]
            else:
                print("Robot %d not connected!" % robotId)
                return
            try:
                realRobot.write(data)
            except:
                del self._real_robots[realRobot]
            
    @tornado.gen.coroutine
    def getAllOrders(self):
        # fetch orders from the database by sending a get request
        request = httpclient.HTTPRequest(self.destination, method="GET")
        response = yield self.http_client.fetch(request)
        db_orders = eval(response.buffer.read()) #I'm sorry Kosbie
        orders = []
        for (id, userId, drinkId, completed, time, robotId, priority) in db_orders:
            robot = None if robotId == -1 else self.robots[robotId]
            orders.append(Order(id, userId, drinkId, completed=completed, 
                                time=time, robot=robot, priority=priority))
        return orders

    def updateAssignedOrders(self, allOrders):
        for robotId in self.robots:
            robot = self.robots[robotId]
            for assignedOrder in robot.getOrders():
                for newOrder in allOrders:
                    if newOrder.id == assignedOrder.id:
                        assignedOrder.completed = newOrder.completed
                        assignedOrder.priority = newOrder.priority

    # sort robots by distance remaining in their trip
    def sortRobots(self):
        robotDistances = dict()
        for robotId in self.robots:
            robot = self.robots[robotId]
            if self.bartenderId in self._loc.locations:
                position = self._loc.locations[self.bartenderId]
                (barX, barY, barZ) = (position.x, position.y, position.z)
            else:
                (barX, barY, barZ) = (0,0,0)
        
            robotDistances[robot.id] = robot.getTripDistance(barX, barY, self._loc)
        return sorted(robotDistances.keys(), key=lambda robot: robotDistances[robot])

    # sort orders by distance to firstOrder
    def getClosestOrders(self, firstOrder, orders):
        (fx, fy, fz) = firstOrder.getLocation(self._loc)
        return sorted(orders, key=lambda order: distance(fx, fy, fz, *order.getLocation(self._loc)))

    def assignRestOfOrders(self, robot, uncompletedOrders, finalQueue, robotOrders):
        remainingOrders = self.getClosestOrders(robotOrders[0], uncompletedOrders)
        remainingOrders = remainingOrders[0 : robot.capacity - len(robotOrders)]
        robotOrders.extend(remainingOrders)
        for assignedOrder in remainingOrders:
            uncompletedOrders.remove(assignedOrder)
            assignedOrder.robot = robot
            assignedOrder.priority = len(finalQueue)
            finalQueue.append(assignedOrder)

    def assignOrders(self, uncompletedOrders):
        robotQueue = self.sortRobots()
        finalQueue = []
        firstPass = True
        while (len(uncompletedOrders) > 0):
            for robotId in robotQueue:
                if len(uncompletedOrders) == 0: break
                robot = self.robots[robotId]
                if firstPass and not robot.inTransit:
                    robotOrders = robot.getOrders()
                    # don't reassign orders already assigned to it
                    for assignedOrder in robotOrders:
                        if assignedOrder in uncompletedOrders:
                            uncompletedOrders.remove(assignedOrder)
                        else: # order must have been completed
                            assignedOrder.completed = True
                        assignedOrder.priority = len(finalQueue)
                        finalQueue.append(assignedOrder)
                    if len(robotOrders) == 0:
                        # assign it the first one
                        firstOrder = uncompletedOrders.pop(0)
                        firstOrder.robot = robot
                        firstOrder.priority = len(finalQueue)
                        robotOrders.append(firstOrder)
                        finalQueue.append(firstOrder)
                    self.assignRestOfOrders(robot, uncompletedOrders, 
                                            finalQueue, robotOrders)
                else:
                    # temporarily assign orders to remaining robots
                    firstOrder = uncompletedOrders.pop(0)
                    firstOrder.robot = robot
                    firstOrder.priority = len(finalQueue)
                    finalQueue.append(firstOrder)
                    robotOrders = [firstOrder]
                    self.assignRestOfOrders(robot, uncompletedOrders, 
                                            finalQueue, robotOrders)
            firstPass = False
        return finalQueue

    @tornado.gen.coroutine
    def updateDatabase(self):
        # insert new priorities, robots into database by sending post request
        for order in self.orderQueue:
            robotId = -1 if order.robot == None else order.robot.id
            body = 'command=update&id=%s&robot_id=%s&priority=%s' % (order.id, robotId, order.priority)
            request = httpclient.HTTPRequest(self.destination, body=body, method="POST")
            response = yield self.http_client.fetch(request)
       
    def intersects(self, robot, destX, destY, destZ):
        # is the robot at the destination?
        (robotX, robotY, robotZ) = robot.getLocation(self._loc)
        return distance(robotX, robotY, robotZ, destX, destY, destZ) <= self.thresh

    @tornado.gen.coroutine
    def deleteOrder(self, order):
        body = 'command=delete&id=%s' % order.id
        request = httpclient.HTTPRequest(self.destination, body=body, method="POST")
        response = yield self.http_client.fetch(request)
        print("order deleted")

    async def setRobotGoals(self):
        for robotId in self.robots:
            robot = self.robots[robotId]
            robotOrders = robot.getOrders()
            if not robot.inTransit:
                robot.isReady()
            else:
                if len(robotOrders) > 0:
                    # move towards next order
                    targetOrder = robotOrders[0]
                    (tX, tY, tZ) = targetOrder.getLocation(self._loc)
                    robot.goal = (tX, tY, tZ)
                    if self.intersects(robot, tX, tY, tZ):
                        if robot.goalTime == None:
                            # wait until we've been near the customer for a while
                            print("Near customer! Waiting %d seconds" 
                                    % robot.waitTime)
                            robot.goalTime = time.time()
                        elif time.time() - robot.goalTime >= robot.waitTime:
                            # customer should have it by now, mark the order as delivered
                            print("deleting order %d" % targetOrder.id)
                            await self.deleteOrder(targetOrder)
                            robotOrders.pop(0)
                            robot.goalTime = None
                        else:
                            print("Near customer! Waiting %d more seconds" %
                                (robot.waitTime - (time.time() - robot.goalTime)))
                    else:
                        robot.goalTime = None
                else: # go back to the bar and refill
                    if self.bartenderId in self._loc.locations:
                        position = self._loc.locations[self.bartenderId]
                        (barX, barY, barZ) (position.x, position.y, position.z)
                    else:
                        (barX, barY, barZ) = (0,0,0)
        
                    if self.intersects(robot, barX, barY, barZ):
                        robot.inTransit = False
                        robot.goal = None
                    else:
                        robot.goal = (barX, barY, barZ)
                print(robot.goal)

    async def updateScheduler(self):
        allOrders = await self.getAllOrders()
        self.updateAssignedOrders(allOrders)
        uncompletedOrders = list(filter(lambda order: not order.completed, allOrders))
        newQueue = self.assignOrders(uncompletedOrders)
        if newQueue != self.orderQueue:
            self.orderQueue = newQueue
            await self.updateDatabase()
        print(newQueue)
        await self.setRobotGoals()

if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    ioloop = tornado.ioloop.IOLoop.instance()
    scheduler = Scheduler(ioloop)
    scheduler.listen(4242)
    logging.info("Starting scheduler!")

    tornado.ioloop.PeriodicCallback(scheduler.updateScheduler, 1000).start()
    logging.info("Starting robot updater!")
    tornado.ioloop.PeriodicCallback(scheduler.update_robots, 250).start()
    ioloop.start()
