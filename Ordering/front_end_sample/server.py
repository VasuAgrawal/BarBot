#!/usr/bin/env python3
import logging
import json

from Order import Order
from Drink import Drink
from datetime import datetime

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

class RootHandler(PostgresHandler):
    def get(self):
        user = self.get_current_user()
        if(user):
            if(user in self.application.bartender):
                self.redirect("/bartender/")
            else:
                self.redirect("/customer/")
        else:
            self.redirect("/login/")

class LoginHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self):
        self.render("static/html/login.html")

    @tornado.gen.coroutine
    def post(self):
        email = str(self.get_argument("Email"))
        password = str(self.get_argument("Password"))
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
            self.redirect("/setup/")

class RegisterHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self):
        self.render("static/html/register.html")

    @tornado.gen.coroutine
    def post(self):
        name = str(self.get_argument("Name"))
        email = str(self.get_argument("Email"))
        password = str(self.get_argument("Password"))
        wristband_id = str(self.get_argument("WristbandID"))
        if(len(name) > 0 and len(email) > 0 and len(password) > 0 and wristband_id.isdigit()):
            sql = """
                    INSERT INTO users(name, email, password, wristband_id)
                    VALUES(%s, %s, %s, %s)
                """
            cursor = yield self.db().execute(sql, (name, email,password,int(wristband_id)))
            self.redirect("/")
        else:
            self.write("Register Fail")
            self.finish()


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
    @tornado.gen.coroutine
    def get(self):
        sql = """
                SELECT id, name, price
                FROM drinks
            """
        cursor = yield self.db().execute(sql)
        desc = cursor.description
        result = [dict(zip([col[0] for col in desc], row)) for row in cursor.fetchall()]

        drinks = []
        for item in result:
            drink = Drink(item['id'], item['name'], item['price'])
            drinks.append(drink)

        # get customer's current orders
        sql = """
            SELECT id, drink_id, completed, time
            FROM orders
            WHERE user_id=%s
        """
        cursor = yield self.db().execute(sql, (self.get_current_user(),))
        result = cursor.fetchall()
        currentOrders = []
        for (id, drink_id, completed, time) in result:
            drink_name = ""
            for drink in drinks:
                if drink_id == drink.id: drink_name = drink.type
            currentOrders.append(Order(id=id, drinkType=drink_name, completed=completed, time=time))
        
        self.render("static/html/customer.html", drinks=drinks, orders=currentOrders)

class MenuHandler(PostgresHandler):
    @tornado.web.authenticated
    @tornado.gen.coroutine
    def post(self):
        user = self.get_current_user()
        if(user not in self.application.bartender):
            self.redirect("/")
        else:
            id = self.get_argument("drinkId", default=None)
            if(id):
                id = int(id)
                sql = """
                        DELETE FROM drinks WHERE id = %s;
                    """
                cursor = yield self.db().execute(sql, (id, ))
                self.redirect("/bartender/")
            drinkType = self.get_argument("drinkType", default=None)
            price = self.get_argument("price", default=None)
            if(drinkType and price):
                drinkType = str(drinkType)
                price = float(price)
                sql = """ INSERT INTO drinks(name, price)
                          VALUES(%s, %s)
                    """
                cursor = yield self.db().execute(sql, (drinkType, price))
                self.redirect("/")

class BartenderHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self):
        user = self.get_current_user()
        if(user not in self.application.bartender):
            self.redirect("/")
        else:
            drinks = []
            orders = []

            drink_sql = """
                    SELECT id, name, price
                    FROM drinks
                """
            drink_cursor = yield self.db().execute(drink_sql)
            drink_desc = drink_cursor.description
            drink_result = [dict(zip([col[0] for col in drink_desc], row)) for row in drink_cursor.fetchall()]

            for item in drink_result:
                drink = Drink(item['id'], item['name'], item['price'])
                drinks.append(drink)

            order_sql = """
                    SELECT id, wristband_id, drink_id, completed, time, robot_id, priority
                    FROM orders
                    WHERE completed = FALSE
                    ORDER BY priority
                """
            order_cursor = yield self.db().execute(order_sql)
            drink_desc = order_cursor.description
            order_result = [dict(zip([col[0] for col in drink_desc], row)) for row in order_cursor.fetchall()]

            for item in order_result:
                drinkId = item['drink_id']
                drinkName = ""
                for drink in drinks:
                    if(drink.id == drinkId):
                        drinkName = drink.type
                order = Order(item['id'], item['wristband_id'], drinkName, item['completed'], item['time'], item['robot_id'])
                orders.append(order)
            self.render("static/html/bartender.html", orders=orders, drinks=drinks)

    @tornado.gen.coroutine
    def post(self):
        id = self.get_argument("orderId", default=None)
        print("marking as complete!", id)
        if(id):
            id = int(id)
            sql = """
                UPDATE orders
                SET completed=TRUE
                WHERE id=%s;
            """
            cursor = yield self.db().execute(sql, (id, ))
        self.redirect("/")

# cancel an order
class ApiCancelHandler(PostgresHandler):
    @tornado.web.authenticated
    @tornado.gen.coroutine
    def post(self):
        order_id = self.get_argument("orderId", default = None)
        if order_id:
            sql = """
                DELETE FROM orders where id=%s
            """
            print("canceling order %s" % order_id)
            cursor = yield self.db().execute(sql, (order_id,))
        self.redirect("/customer")

# Insert an order into the database
class ApiOrderHandler(PostgresHandler):
    @tornado.web.authenticated
    @tornado.gen.coroutine
    def post(self):
        drink_id = self.get_argument("drinkId", default = None)
        if(drink_id):
            # get wristband_id associated with this user
            user_id = self.get_current_user()
            wristband_sql = """
                SELECT wristband_id
                FROM users
                WHERE id=%s
            """
            wristband_cursor = yield self.db().execute(wristband_sql, (user_id,))
            wristband_id = wristband_cursor.fetchall()[0]

            dt = datetime.now()
            drink_id = int(drink_id)
            sql ="""
                INSERT INTO orders(user_id, wristband_id, drink_id, completed, time, robot_id, priority)
                VALUES (%s, %s, %s, FALSE, %s, %s, %s)
                """
            order_cursor = yield self.db().execute(sql, (user_id, wristband_id, drink_id, dt, "-1", 1000))
            self.redirect("/customer")
        else:
            self.write("Order Failed!")
            self.finish()

#Keeping init scripts here, may not be a great idea
class SetUpHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self):
        print("initializing")
        # USER TABLE INIT
        user_sql = """
                CREATE SEQUENCE IF NOT EXISTS user_id;
                CREATE TABLE IF NOT EXISTS users (
                    id integer PRIMARY KEY DEFAULT nextval('user_id') ,
                    name  varchar(80),
                    email  varchar(80) UNIQUE,
                    password  varchar(80),
                    wristband_id integer
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
                    wristband_id integer,
                    completed boolean DEFAULT FALSE,
                    time timestamp,
                    robot_id integer DEFAULT -1,
                    priority integer
                );
                ALTER SEQUENCE order_id OWNED BY orders.id;
            """
        order_cursor = yield self.db().execute(order_sql)


        self.redirect("/")

class ResetHandler(PostgresHandler):
    @tornado.gen.coroutine
    @tornado.web.authenticated
    def get(self):
        print("reseting")
        delete_sql = """
            DROP SEQUENCE IF EXISTS user_id CASCADE;
            DROP TABLE IF EXISTS users CASCADE;
            DROP TABLE IF EXISTS drinks CASCADE;
            DROP TABLE IF EXISTS orders CASCADE;
        """

        delete_cursor = yield self.db().execute(delete_sql)

        self.redirect("/setup")
        
class SchedulerHandler(PostgresHandler):
    @tornado.gen.coroutine
    def get(self):
        order_sql = """
            SELECT id, wristband_id, drink_id, completed, time, robot_id, priority
            FROM orders
            ORDER BY time
        """

        order_cursor = yield self.db().execute(order_sql)

        self.write("%r" % (order_cursor.fetchall(),))
        self.finish()
    
    @tornado.gen.coroutine
    def post(self):
        id = int(self.get_argument("id"))
        robot_id = int(self.get_argument("robot_id"))
        priority = int(self.get_argument("priority"))
        print("updating database: %d, %d, %d" % (id, robot_id, priority))
        sql = """
            UPDATE orders
            SET robot_id=%s, priority=%s
            WHERE id=%s
        """

        order_cursor = yield self.db().execute(sql, (robot_id, priority, id))

        self.write("Done!\n")
        self.finish()

    @tornado.gen.coroutine
    def delete(self):
        id = int(self.get_argument("id"))
        sql = """
            DELETE FROM orders WHERE id = %s;
        """

        order_cursor = yield self.db().execute(sql, (id, ))

        self.write("Done!\n")
        self.finish()
        
class BarBotApplication(tornado.web.Application):
    def __init__(self, ioloop):
        logging.info("Starting logging!")
        logging.root.setLevel(logging.DEBUG)

        handlers = [
            #TODO tune these regex
            (r"/?", RootHandler),
            (r"/setup/?", SetUpHandler),
            (r"/reset/?", ResetHandler),
            (r"/customer/?", CustomerHandler),
            (r"/bartender/?", BartenderHandler),
            (r"/menu/?", MenuHandler),
            (r"/v0/cancel/", ApiCancelHandler),
            (r"/v0/order/", ApiOrderHandler),
            (r"/scheduler/", SchedulerHandler),
            (r"/login/?", LoginHandler),
            (r"/register/?", RegisterHandler),
            (r"/about/?", AboutHandler),
            (r"/logout/?", LogoutHandler)
        ]

        settings = {
            "static_path": "static",
            "cookie_secret": "TODO some real cookie secret",
            "login_url": "/login/",
            "debug": True,
            "autoreload": True,
        }
        tornado.web.Application.__init__(self, handlers, **settings)

        #dsn = 'dbname=template1 user=Kim password=icanswim ' \
        #         'host=localhost port=10601'
        dsn = 'dbname=barbotdb user=barbotdev password=icanswim host=localhost port=10601'
        #dsn = 'dbname=template1 user=postgres ' \

        self.db = momoko.Pool(dsn=dsn, size=2, ioloop=ioloop)
        self.bartender = [1,2]


def main():
    ioloop = tornado.ioloop.IOLoop.instance()
    app = BarBotApplication(ioloop)
    
    dbConnection = app.db.connect()
    ioloop.add_future(dbConnection, lambda f: ioloop.stop())
    ioloop.start()
    dbConnection.result()

    app.listen(8080)
    ioloop.start()

if __name__ == "__main__":
    main()
