#!/usr/bin/env python3

from datetime import datetime
import math
from Order import Order
import psycopg2
import random
from Robot import Robot
from tkinter import *

def init(data):
    initDb(data)

    data.locations = dict()
    data.orderQueue = []
    data.allOrders = []

    initSim(data)

def initDb(data):
    sql = """
        DELETE FROM orders
    """
    doADbThing(sql, ())
    
def doADbThing(sql, args, fetch=False):
    con = psycopg2.connect(
        dbname="template1",
        user="postgres",
        # password="icanswim",
        host="localhost",
        port=10601)

    cur = con.cursor()
    if len(args) > 0:
        cur.execute(sql, args)
    else:
        cur.execute(sql)

    if fetch:
        res = cur.fetchall()
    else:
        res = []

    con.commit()
    cur.close()

    return res

def initSim(data):
    data.barSize = 80
    data.barX = data.barSize / 2
    data.barY = data.barSize / 2
    data.orderRadius = 10
    data.robotSize = 20
    data.numRobots = 2
    data.robotSpeed = 5
    data.robots = []
    for i in range(data.numRobots):
        data.robots.append(Robot(i, location=(0,0)))

def getAllOrders(data):
    order_sql = """
            SELECT id, user_id, drink_id, completed, time, robot_id, priority
            FROM orders
            ORDER BY time
        """
    newOrders = doADbThing(order_sql, (), True)
    
    orders = []
    for (id, userId, drinkId, completed, time, robotId, priority) in newOrders:
        robot = None if robotId == -1 else data.robots[robotId]

        #TODO: remove this when we hook it up to the GLS
        if id not in data.locations:
            data.locations[id] = (random.randint(data.barSize, data.width), random.randint(data.barSize, data.height))

        orders.append(Order(id, userId, drinkId, completed=completed, time=time, robot=robot, priority=priority))
    return orders

def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# sort robots by distance remaining in their trip
def sortRobots(data):
    robotDistances = dict()
    for robot in data.robots:
        robotDistances[robot.id] = robot.getTripDistance(data)
    return sorted(robotDistances.keys(), key=lambda robot: robotDistances[robot])

def getClosestOrders(data, firstOrder, orders):
    (fx, fy) = firstOrder.getLocation(data)
    return sorted(orders, key=lambda order: distance(fx, fy, *order.getLocation(data)))

def assignRestOfOrders(data, robot, uncompletedOrders, finalQueue, robotOrders):
    remainingOrders = getClosestOrders(data, robotOrders[0], uncompletedOrders)[0 : robot.capacity - len(robotOrders)]
    robotOrders.extend(remainingOrders)
    for assignedOrder in remainingOrders:
        uncompletedOrders.remove(assignedOrder)
        assignedOrder.robot = robot
        assignedOrder.priority = len(finalQueue)
        finalQueue.append(assignedOrder)
    
def assignOrders(data, uncompletedOrders):
    robotQueue = sortRobots(data)
    finalQueue = []
    firstPass = True
    while (len(uncompletedOrders) > 0):
        for robotId in robotQueue:
            if len(uncompletedOrders) == 0:
                break
            robot = data.robots[robotId]
            if firstPass and not robot.inTransit:
                robotOrders = robot.getOrders()
                #don't reassign orders already assigned to it
                for assignedOrder in robotOrders:
                    if assignedOrder in uncompletedOrders:
                        uncompletedOrders.remove(assignedOrder)
                    else: #order must have been completed
                        #print("marking as completed!")
                        assignedOrder.completed = True
                    assignedOrder.priority = len(finalQueue)
                    finalQueue.append(assignedOrder)
                if len(robotOrders) == 0:
                    #assign it the first one
                    firstOrder = uncompletedOrders.pop(0)
                    firstOrder.robot = robot
                    firstOrder.priority = len(finalQueue)
                    robotOrders.append(firstOrder)
                    finalQueue.append(firstOrder)
                assignRestOfOrders(data, robot, uncompletedOrders, finalQueue, robotOrders)
            else:
                #temporarily assign orders to remaining robots
                firstOrder = uncompletedOrders.pop(0)
                firstOrder.robot = robot
                firstOrder.priority = len(finalQueue)
                finalQueue.append(firstOrder)
                robotOrders = [firstOrder]
                assignRestOfOrders(data, robot, uncompletedOrders, finalQueue, robotOrders)
        firstPass = False
    return finalQueue

def updateDatabase(data):
    #insert new priorities, robots
    for order in data.orderQueue:
        sql = """
            UPDATE orders
            SET robot_id=%s, priority=%s
            WHERE id=%s
        """
        robotId = -1 if order.robot == None else order.robot.id
        doADbThing(sql, (robotId, order.priority, order.id))

def intersects(data, robot, destX, destY):
    (robotX, robotY) = robot.getLocation()
    return (robotX - data.robotSize/2 <= destX <= robotX + data.robotSize/2
        and robotY - data.robotSize/2 <= destY <= robotY + data.robotSize/2)

def moveRobots(data):
    for robot in data.robots:
        robotOrders = robot.getOrders()
        if not robot.inTransit:
            robot.isReady()
        else:
            if len(robotOrders) > 0:
                #move towards next order
                targetOrder = robotOrders[0]
                (tX, tY) = targetOrder.getLocation(data)
                robot.move(tX, tY)
                if intersects(data, robot, tX, tY):
                    #remove it from the database
                    sql = """
                        DELETE FROM orders WHERE id = %s;
                    """
                    doADbThing(sql, (targetOrder.id, ))
                    robotOrders.pop(0)
            else: #go back to bar and refill
                if intersects(data, robot, data.barSize/2, data.barSize/2):
                    robot.inTransit = False
                else:
                    robot.move(data.barX, data.barY)

def updateAssignedOrders(data):
    for robot in data.robots:
        for assignedOrder in robot.getOrders():
            for newOrder in data.allOrders:
                if newOrder.id == assignedOrder.id:
                    assignedOrder.completed = newOrder.completed
                    assignedOrder.priority = newOrder.priority

#TODO: remove all orders on a robot before prioritizing
def timerFired(data):
    data.allOrders = getAllOrders(data)
    updateAssignedOrders(data)
    #print("all orders", data.allOrders)
    #for robot in data.robots:
     #   print(robot.id, robot.getOrders())
    uncompletedOrders = list(filter(lambda order: order.completed==False, data.allOrders))
    newQueue = assignOrders(data, uncompletedOrders)
    if newQueue != data.orderQueue:
        data.orderQueue = newQueue
        #print(data.orderQueue)
        updateDatabase(data)
    moveRobots(data)

def mousePressed(event, data):
    pass

def keyPressed(event, data):
    pass

def drawBar(canvas, data):
    canvas.create_rectangle(0, 0, data.barSize, data.barSize, fill="brown")
    canvas.create_text(data.barSize/2, data.barSize/2, text="BAR")

def drawRobots(canvas, data):
    for robot in data.robots:
        (x, y) = robot.getLocation()
        fill = "blue" if robot.id == 0 else "green"
        canvas.create_rectangle(x - data.robotSize, y - data.robotSize,
            x + data.robotSize, y + data.robotSize, fill=fill)

def drawOrders(canvas, data):
    for order in data.allOrders:
        (x, y) = order.getLocation(data)
        if order.robot == None:
            fill = "black"
        elif order.robot.id == 0:
            fill = "blue"
        else:
            fill = "green"
        canvas.create_oval(x - data.orderRadius, y - data.orderRadius, 
            x + data.orderRadius, y + data.orderRadius, fill=fill)
        canvas.create_text(x, y, text=order.id)

def redrawAll(canvas, data):
    drawBar(canvas, data)
    drawRobots(canvas, data)
    drawOrders(canvas, data)

def run(width=300, height=300):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, data.width, data.height,
                                fill='white', width=0)
        redrawAll(canvas, data)
        canvas.update()    

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
    # Set up data and call init
    class Struct(object): pass
    data = Struct()
    data.width = width
    data.height = height
    data.timerDelay = 100 # milliseconds
    init(data)
    # create the root and the canvas
    root = Tk()
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.pack()
    # set up events
    root.bind("<Button-1>", lambda event:
                            mousePressedWrapper(event, canvas, data))
    root.bind("<Key>", lambda event:
                            keyPressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed
    print("bye!")

run(600, 600)
