import os
import signal
import tornado.ioloop
import tornado.web

root = os.path.dirname(os.path.realpath(__file__)) + "/../www"
port = 8080

class MyApplication(tornado.web.Application):
    is_closing = False

    def signal_handler(self, signum, frame):
        self.is_closing = True

    def try_exit(self):
        if self.is_closing:
            print('exiting')
            tornado.ioloop.IOLoop.instance().stop()

application = MyApplication([
    (r"/(.*)", tornado.web.StaticFileHandler, {"path": root, "default_filename": "index.html"})
])

signal.signal(signal.SIGINT, application.signal_handler)
application.listen(port)
tornado.ioloop.PeriodicCallback(application.try_exit, 100).start()

print(f"root: {root}")
print(f"started: http://127.0.0.1:{port}")

tornado.ioloop.IOLoop.instance().start()
