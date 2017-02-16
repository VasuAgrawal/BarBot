class OrderQueue(object):

    def __init__(self):
        self.queue = []

    def enqueue(self, order):
        self.queue.append(order)

    def dequeue(self):
        if self.size() > 0:
            return self.queue.pop(0)
        raise Exception("Cannot dequeue from empty queue")

    def size(self):
        return len(self.queue)