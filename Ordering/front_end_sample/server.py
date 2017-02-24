#!/usr/bin/env python3
import logging
import json

from Order import Order
from Drink import Drink

import tornado.ioloop
import tornado.web

orders = []
drinks = [Drink("Beer", 3), Drink("Wine", 4)]

class BaseHandler(tornado.web.RequestHandler):
    def get_current_user(self):
        return self.get_secure_cookie("user")

class RootHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/index.html")

class LoginHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/login.html")

class LogoutHandler(tornado.web.RequestHandler):
    def get(self):
        self.redirect("/")

class AboutHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/html/about.html")

class CustomerHandler(tornado.web.RequestHandler):
    #@tornado.web.authenticated # Example of authentication
    def get(self):
        print("Doing customer handler!")
        # TODO figure out how to not hard code "static" here
        self.render("static/html/customer.html", drinks=drinks)

    def post(self):
        print("got post request!")
        print(self.get_argument("Email"), self.get_argument("Password"))
        self.redirect("/customer/")

class BartenderHandler(tornado.web.RequestHandler):
    def get(self):
        # TODO figure out how to not hard code "static" here
        self.render("static/html/bartender.html", orders=orders)

    def post(self):
        print("got a bartender post request")
        id = int(self.get_argument("orderId"))
        print("removing order %d" % id)
        for order in orders:
            if order.id == id:
                orders.remove(order)
                break
        self.redirect("/bartender/")



class ApiDrinkHandler(tornado.web.RequestHandler):
    def get(self):
        print(self.get_argument("id", default=None))
        self.write(json.dumps({"drink1": "beer"}))

    def post(self):
        print(self.get_argument("id"))


class ApiOrderHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({"order1": "some order"}))

    def post(self):
        for drink in drinks:
            if self.get_argument(drink.type + "-checkbox", default=None) != None:
                orders.append(Order("Bob", drink.type, 0))
        self.redirect("/customer/")


class ApiRobotHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({"robot": "robot data!"}))


class ApiCustomerHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({"customer": "customer data"}))

def main():
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
    
    tornado.web.Application(handlers, **settings).listen(8888)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
