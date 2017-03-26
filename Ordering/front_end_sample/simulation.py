from tkinter import *
from Order import Order
from scheduler import Scheduler
from datetime import datetime
import math

def init(data):
    data.barSize = 80
    data.orderRadius = 10
    data.robotSize = 40
    data.numRobots = 2
    data.scheduler = Scheduler(data.numRobots)
    data.robotSpeed = 5
    data.numOrders = 0

def mousePressed(event, data):
    orderID = data.numOrders
    data.numOrders += 1
    time = datetime.now()
    location = (event.x, event.y)
    order = Order(orderID, "", "", "", time, location=location)
    data.scheduler.addOrder(order)

def keyPressed(event, data):
    if event.keysym == "r":
        init(data)

def intersects(data, robot, destX, destY):
    (robotX, robotY) = robot.getLocation()
    return (robotX - data.robotSize/2 <= destX <= robotX + data.robotSize/2
        and robotY - data.robotSize/2 <= destY <= robotY + data.robotSize/2)

def moveRobot(data, robot, destX, destY):
    (robotX, robotY) = robot.getLocation()
    dx, dy = destX - robotX, destY - robotY
    angle = math.atan2(dy, dx)
    dx = math.cos(angle) * data.robotSpeed
    dy = math.sin(angle) * data.robotSpeed
    robot.updateLocation((robotX + dx, robotY + dy))    

def timerFired(data):
    for robot in data.scheduler.robots:
        orders = robot.getOrders()
        if len(orders) != 0:
            robot.inTransit = True
            targetOrder = orders[0]
            (orderX, orderY) = targetOrder.getLocation()
            moveRobot(data, robot, orderX, orderY)
            if intersects(data, robot, orderX, orderY):
                data.scheduler.removeOrder(targetOrder)
        else: #go back to bar and refill
            if intersects(data, robot, data.barSize/2, data.barSize/2):
                robot.inTransit = False
                data.scheduler.updateRobot(robot)
            else:
                moveRobot(data, robot, data.barSize/2, data.barSize/2)

def drawOrders(canvas, data):
    for order in data.scheduler.getOrders():
        (x, y) = order.getLocation()
        if order.robot == None:
            color = "black"
        elif order.robot.id == 0:
            color = "blue"
        else:
            color = "green"
        canvas.create_oval(x-data.orderRadius, y-data.orderRadius, 
            x+data.orderRadius, y+data.orderRadius, fill=color)

def drawRobots(canvas, data):
    for robot in data.scheduler.robots:
        (x, y) = robot.getLocation()
        color = "blue" if robot.id == 0 else "green"
        canvas.create_rectangle(x - data.robotSize/2, y - data.robotSize/2,
            x + data.robotSize/2, y + data.robotSize/2, fill=color)

def redrawAll(canvas, data):
    canvas.create_rectangle(0, 0, data.barSize, data.barSize, fill="brown")
    canvas.create_text(data.barSize/2, data.barSize/2, text="Bar")
    drawOrders(canvas, data)
    drawRobots(canvas, data)

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