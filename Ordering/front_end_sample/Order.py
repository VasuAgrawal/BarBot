class Order(object):
    orderID = 0 # so we can assign a unique id to each order

    def __init__(self, customer=None, orderType=None, robot=None):
        self.id = Order.orderID
        Order.orderID += 1
        self.customer = customer
        self.orderType = orderType
        self.robot = robot

    def getID(self):
        return self.id

    def setCustomer(self, customer):
        self.customer = customer

    def getCustomer(self):
        return self.customer

    def setOrderType(self, orderType):
        self.orderType = orderType

    def getOrderType(self):
        return self.orderType

    def __eq__(self, other):
        return self.id == other.id

    def __repr__(self):
        return "Order %d: %r %s" % (self.id, self.customer, self.orderType)