class Drink(object):

    drinkId = 0

    def __init__(self, drinkType, price):
        self.type = drinkType
        self.price = price
        self.id = Drink.drinkId
        Drink.drinkId += 1