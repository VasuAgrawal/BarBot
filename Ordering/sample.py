from Order import Order
from Customer import Customer
from OrderTypes import OrderType
from OrderQueue import OrderQueue

bob = Customer(1)
joe = Customer(2)

queue = OrderQueue()

ord1 = Order(bob, OrderType.BEER)
queue.enqueue(ord1)
ord2 = Order(joe, OrderType.WINE)
queue.enqueue(ord2)

print(queue.dequeue())
print(queue.dequeue())