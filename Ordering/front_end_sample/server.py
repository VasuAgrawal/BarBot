#!/usr/bin/env python3
import logging
import json

from Order import Order
from Drink import Drink

import tornado.gen
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

    def get_current_user(self):
        id = self.get_secure_cookie("id")
        if(id):
            return int(id)
        else:
            return None


class BaseHandler(PostgresHandler):
    def get_current_user(self):
        return self.get_secure_cookie("user")

class RootHandler(PostgresHandler):
    def get(self):
        self.render("static/html/index.html")

class LoginHandler(PostgresHandler):
    def get(self):
        self.render("static/html/login.html")

    @tornado.gen.coroutine
    def post(self):
        email = str(self.get_argument("Email"))
        password = str(self.get_argument("Password"))
        print("received log in request")
        print(email, password)
        sql = """
                SELECT id, name, email
                FROM users
                WHERE email=%s AND password=%s;
            """
        cursor = yield self.db().execute(sql, (email,password,))
        desc = cursor.description
        result = [dict(zip([col[0] for col in desc], row)) for row in cursor.fetchall()]
        cursor.close()

        if(len(result) == 0):
            self.write("Log In Fail!")
            self.finish()
        else: 
            user = result[0]
            self.set_secure_cookie("name", user["name"])
            self.set_secure_cookie("id", str(user["id"]))

            print(self.get_current_user())
            self.redirect("/")

        #self.redirect("/")

class LogoutHandler(PostgresHandler):
    def get(self):
        self.clear_cookie("name")
        self.clear_cookie("id")
        self.redirect("/")

class AboutHandler(PostgresHandler):
    def get(self):
        self.render("static/html/about.html")

class CustomerHandler(PostgresHandler):
    @tornado.web.authenticated # Example of authentication
    def get(self):
        print("Doing customer handler!")
        # TODO figure out how to not hard code "static" here
        self.render("static/html/customer.html", drinks={})

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


#Keeping init scripts here, may not be a great idea
class SetUpHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self):
        self.write("What are you doing here?")
        self.finish()

    @tornado.gen.coroutine
    def post(self):
        
        # USER TABLE INIT
        user_sql = """
                CREATE SEQUENCE IF NOT EXISTS user_id;
                CREATE TABLE IF NOT EXISTS users (
                    id integer PRIMARY KEY DEFAULT nextval('user_id') ,
                    name  varchar(80) UNIQUE,
                    email  varchar(80) UNIQUE,
                    password  varchar(80) 
                );
                ALTER SEQUENCE user_id OWNED BY users.id;
            """
        user_cursor = yield self.db().execute(user_sql)

        # DRINKS MENU INIT
        drink_sql = """
                CREATE SEQUENCE IF NOT EXISTS drink_id;
                CREATE TABLE IF NOT EXISTS drinks (
                    id integer PRIMARY KEY DEFAULT nextval('drink_id') ,
                    name  varchar(80) UNIQUE,
                    price real 
                );
                ALTER SEQUENCE drink_id OWNED BY drinks.id;
            """
        drink_cursor = yield self.db().execute(drink_sql)

        # ORDERS INIT
        order_sql = """
                CREATE SEQUENCE IF NOT EXISTS order_id;
                CREATE TABLE IF NOT EXISTS orders (
                    id integer PRIMARY KEY DEFAULT nextval('order_id'),
                    user_id integer REFERENCES users(id) ON UPDATE CASCADE ON DELETE CASCADE,
                    drink_id integer REFERENCES drinks(id) ON UPDATE CASCADE ON DELETE CASCADE,
                    quantity integer NOT NULL DEFAULT 1,
                    completed boolean DEFAULT FALSE,
                    time timestamp
                );
                ALTER SEQUENCE order_id OWNED BY orders.id;
            """
        order_cursor = yield self.db().execute(order_sql)

        self.write("DONE\n")
        self.finish()

class UserHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self, id=None):
        if not id:
            sql = """
                SELECT id, name, email, password
                FROM users;
            """
            cursor = yield self.db().execute(sql)
            desc = cursor.description
            result = [dict(zip([col[0] for col in desc], row)) for row in cursor.fetchall()]
            cursor.close()

            self.write(json.dumps(result))
            self.finish()

class BatBotApplication(tornado.web.Application):
    def __init__(self, ioloop):
        logging.info("Starting logging!")
        logging.root.setLevel(logging.DEBUG)

        handlers = [
            #TODO tune these regex
            (r"/?", RootHandler),
            (r"/user/?", UserHandler),
            (r"/setup/", SetUpHandler),
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
            "login_url": "/login/",
            "debug": True,
            "autoreload": True,
        }

        tornado.web.Application.__init__(self, handlers, **settings)

        dsn = 'dbname=barbotdb user=barbotdev password=icanswim ' \
                  'host=localhost port=5432'

        self.db = momoko.Pool(dsn=dsn, size=1, ioloop=ioloop)
        self.orders = []
        self.drinks = [Drink("Beer", 3), Drink("Wine", 4)]


def main():
    ioloop = tornado.ioloop.IOLoop.instance()
    app = BatBotApplication(ioloop)
    
    dbConnection = app.db.connect()
    ioloop.add_future(dbConnection, lambda f: ioloop.stop())
    ioloop.start()
    dbConnection.result()

    app.listen(8888)
    ioloop.start()

if __name__ == "__main__":
    main()
