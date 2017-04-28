#!/usr/bin/env python3
import logging

import datetime
import momoko
from Order import Order
from Robot import Robot
import packet

import tornado
import tornado.tcpserver
import tornado.gen
import tornado.ioloop
from tornado import httpclient

from positions_pb2 import Point
from positions_pb2 import Locations
from positions_pb2 import ConnectionRequest

class Scheduler(tornado.tcpserver.TCPServer):
    def __init__(self, ioloop, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.http_client = httpclient.AsyncHTTPClient()

        # TODO: update these with actual coordinates of the bar
        self.barX = 0
        self.barY = 0
        self.numRobots = 2
        self.robots = []
        for i in range(self.numRobots):
            #TODO: replace this location with location from GLS
            self.robots.append(Robot(i, location=(0,0)))
        self.orderQueue = []

        self._real_robots = []
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

            loc = Locations()
            loc.ParseFromString(message_bytes)
            logging.info("Received location data!")
            self._loc = loc
         

    @tornado.gen.coroutine
    def handle_robot(self, stream, address):
        logging.info("Received ROBOT connection request!")
        self._real_robots.append(stream)


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
            self.handle_robot(stream, address)
        else:
            logging.error("Invalid connection request state?")


    def update_robots(self):
        data = packet.make_packet_from_bytes(self._loc.SerializeToString())

        for robot in self._real_robots[::-1]:
            try:
                robot.write(data)
            except Exception:
                self._real_robots.remove(robot)

    @tornado.gen.coroutine
    def getAllOrders(self):
        print("getting orders")
        destination = 'http://localhost:8080/scheduler/'
        request = httpclient.HTTPRequest(destination, method="GET")
        response = yield self.http_client.fetch(request)
        db_orders = eval(response.buffer.read()) #I'm sorry Kosbie
        orders = []
        for (id, userId, drinkId, completed, time, robotId, priority) in db_orders:
            robot = None if robotId == -1 else self.robots[robotId]

            orders.append(Order(id, userId, drinkId, completed=completed, time=time, robot=robot, priority=priority))

        return orders

    def updateAssignedOrders(self, allOrders):
        for robot in self.robots:
            for assignedOrder in robot.getOrders():
                for newOrder in allOrders:
                    if newOrder.id == assignedOrder.id:
                        assignedOrder.completed = newOrder.completed
                        assignedOrder.priority = newOrder.priority

    # sort robots by distance remaining in their trip
    def sortRobots(self):
        robotDistances = dict()
        for robot in self.robots:
            robotDistances[robot.id] = robot.getTripDistance(self.barX, self.barY)
        return sorted(robotDistances.keys(), key=lambda robot: robotDistances[robot])

    def distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def getClosestOrders(self, firstOrder, orders):
        (fx, fy) = firstOrder.getLocation()
        return sorted(orders, key=lambda order: distance(fx, fy, *order.getLocation()))

    def assignRestOfOrders(self, robot, uncompletedOrders, finalQueue, robotOrders):
        remainingOrders = self.getClosestOrders(self, robotOrders[0], uncompletedOrders)
        remainingOrders = remainingOrders[0 : robot.capacity - len(robotOrders)]
        robotOrders.extend(remainingOrders)
        for assignedOrder in remainingOrders:
            uncompletedOrders.remove(assignedOrder)
            assignedOrder.robot = robots
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
                    self.assignRestOfOrders(robot, uncompletedOrders, finalQueue, robotOrders)
                else:
                    # temporarily assign orders to remaining robots
                    firstOrder = uncompletedOrders.pop(0)
                    firstOrder.robot = robot
                    firstOrder.priority = len(finalQueue)
                    finalQueue.append(firstOrder)
                    robotOrders = [firstOrder]
                    self.assignRestOfOrders(robot, uncompletedOrders, finalQueue, robotOrders)
            firstPass = False
        return finalQueue

    def updateDatabase(self):
        # insert new priorities, robots
        for order in self.orderQueue:
            robotId = -1 if order.robot == None else order.robot.id
            body = 'robot_id=%s,id=%s,priority=%s' % (robotId, order.id, order.priority)
            destination = 'http://localhost:8080/scheduler/'
            request = httpclient.HTTPRequest(destination, body=body, method="POST")
            response = yield self.http_client.fetch(request)
        
    def updateScheduler(self):
        print("updating scheduler")
        allOrders = self.getAllOrders()
        print(allOrders)
        self.updateAssignedOrders(allOrders)
        uncompletedOrders = list(filter(lambda order: not order.completed, allOrders))
        newQueue = self.assignOrders(uncompletedOrders)
        if newQueue != self.orderQueue:
            self.orderQueue = newQueue
            self.updateDatabase()
        print(newQueue)

if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    ioloop = tornado.ioloop.IOLoop.instance()
    scheduler = Scheduler(ioloop)
    scheduler.listen(4242)
    logging.info("Starting scheduler!")

    print("about to get orders")
    ioloop.run_sync(scheduler.updateScheduler)
    print("got orders")

    # tornado.ioloop.PeriodicCallback(scheduler.updateScheduler, 1000).start()
    logging.info("Starting robot updater!")
    tornado.ioloop.PeriodicCallback(scheduler.update_robots, 1000).start()
    ioloop.start()
