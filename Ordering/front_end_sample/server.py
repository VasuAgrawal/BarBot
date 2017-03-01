#!/usr/bin/env python3
import logging
import json

from Order import Order
from Drink import Drink

import tornado.ioloop
import tornado.web
import momoko

class PostgresHandler(tornado.web.RequestHandler):
    def prepare(self):
        if self.request.headers.get("Content-Type") == "application/json":
            try:
                self.json_args = json_decode(self.request.body)
            except Exception as error:
                self.finish('invalid request')

    def db(self):
        return self.application.db

class BaseHandler(PostgresHandler):
    def get_current_user(self):
        return self.get_secure_cookie("user")

class RootHandler(PostgresHandler):
    def get(self):
        self.render("static/html/index.html")

class LoginHandler(PostgresHandler):
    def get(self):
        self.render("static/html/login.html")

class LogoutHandler(PostgresHandler):
    def get(self):
        self.redirect("/")

class AboutHandler(PostgresHandler):
    def get(self):
        self.render("static/html/about.html")

class CustomerHandler(PostgresHandler):
    #@tornado.web.authenticated # Example of authentication
    def get(self):
        print("Doing customer handler!")
        # TODO figure out how to not hard code "static" here
        self.render("static/html/customer.html", drinks=self.drinks)

    def post(self):
        print("got post request!")
        print(self.get_argument("Email"), self.get_argument("Password"))
        self.redirect("/customer/")

class BartenderHandler(PostgresHandler):
    def get(self):
        # TODO figure out how to not hard code "static" here
        self.render("static/html/bartender.html", orders=self.orders)

    def post(self):
        print("got a bartender post request")
        id = int(self.get_argument("orderId"))
        print("removing order %d" % id)
        for order in self.orders:
            if order.id == id:
                self.orders.remove(order)
                break
        self.redirect("/bartender/")



class ApiDrinkHandler(PostgresHandler):
    def get(self):
        print(self.get_argument("id", default=None))
        self.write(json.dumps({"drink1": "beer"}))

    def post(self):
        print(self.get_argument("id"))


class ApiOrderHandler(PostgresHandler):
    def get(self):
        self.write(json.dumps({"order1": "some order"}))

    def post(self):
        for drink in self.drinks:
            if self.get_argument(drink.type + "-checkbox", default=None) != None:
                self.orders.append(Order("Bob", drink.type, 0))
        self.redirect("/customer/")


class ApiRobotHandler(PostgresHandler):
    def get(self):
        self.write(json.dumps({"robot": "robot data!"}))


class ApiCustomerHandler(PostgresHandler):
    def get(self):
        self.write(json.dumps({"customer": "customer data"}))

class BatBotApplication(tornado.web.Application):
    def __init__(self):
        logging.info("Starting logging!")
        logging.root.setLevel(logging.DEBUG)

        handlers = [
            #TODO tune these regex
            (r"/?", RootHandler),
            (r"/customer/?", CustomerHandler),
            (r"/bartender/?", BartenderHandler),
            (r"/v0/drink/", ApiDrinkHandler),
            (r"/v0/order/", ApiOrderHandler),
            (r"/v0/robot/", ApiRobotHandler),
            (r"/v0/customer/", ApiCustomerHandler),
            (r"/login/?", LoginHandler),
            (r"/about/?", AboutHandler),
            (r"/logout/?", LogoutHandler),
        ]

        settings = {
            "static_path": "static",
            "cookie_secret": "TODO some real cookie secret",
            "login_url": "/",
            "debug": True,
            "autoreload": True,
        }

        tornado.web.Application.__init__(self, handlers, **settings)
        dsn = 'dbname=barbotdb user=barbotdev password=drinks ' \
                  'host=localhost port=5432'

        self.db = momoko.Pool(dsn=dsn, size=1)
        self.orders = []
        self.drinks = [Drink("Beer", 3), Drink("Wine", 4)]



def main():
    app = BatBotApplication()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
