import tornado.ioloop
import tornado.web

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        items = ["a", "b", "c"]
        self.render("template.html", title="Woo", items=items)

class OrderHandler(tornado.web.RequestHandler):
    def get(self, order):
        self.write("Order: %s" % order)

def make_app():
    return tornado.web.Application([(r"/", MainHandler),
        (r"/order/([a-z]+)", OrderHandler)])

if __name__ == "__main__":
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
