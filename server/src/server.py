from flask import Flask, json
from geolocation.geolocation_service import GeolocationThread
from werkzeug.serving import make_server
from apis import api
import argparse
import threading
import signal
import sys


class ServerThread(threading.Thread):
    """
    Thread for the server. Starting the server this way allows us to kill
    both the server and geolocation (running in its own thread), when we want
    (Using just app.start() would catch a ctrl+c, and not allow us to safely
    close geolocation)
    """

    def __init__(self, app, hostname):
        # 100% a stack overflow answer here. no idea what its doing, but it 
        # successfully starts the server in a separate thread and we're able
        # to control when/how it closes down
        super(ServerThread, self).__init__()
        self.srv = make_server(hostname, 5000, app)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        print("Start server")
        # keep the server up until we specifically tell it to die
        self.srv.serve_forever()

    def shutdown(self):
        self.srv.shutdown() # deathhhhhhhhh

def signal_handler(sig, frame):
    # shut the all our server processes down safely
    # this code is run when a SIGINT aka ctrl+c is thrown
    # probably a better way todo this than globals? 
    print('Shutting Down...')
    global server
    server.shutdown()
    try:
        global geolocation
        geolocation.shutdown()
    except NameError:
        return # geolocation is not defined, and probably not running

def main():
    parser = argparse.ArgumentParser(description="Imaging Server Help")
    parser.add_argument('-H', '--host', metavar='hostname', help='The hostname to publish the server on. For Docker, this should be 0.0.0.0')
    parser.add_argument('-g', action='store_true', help="By default geolocation will run in its own thread when the server starts. If you dont want that, you can turn it off with this flag.")

    args = parser.parse_args()

    hostname = "0.0.0.0"
    if args.host is not None:
        hostname = args.host

    if args.g is not None and args.g:
        print("Starting with NO geolocation")
    
    app = Flask(__name__)
    api.init_app(app)
    #app.start(debug=True, host=hostname)

    global server
    server = ServerThread(app, hostname)

    if not args.g:
        global geolocation
        geolocation = GeolocationThread()

    # now that our server thread is setup, we can configure sigint to properly close it
    signal.signal(signal.SIGINT, signal_handler)

    server.start()
    print("Server Started!")
    print("     Host:: {}:{}".format(hostname, 5000))

    if not args.g:
        geolocation.start()
        print("Geolocation Started!")

if __name__ == '__main__':
    main()
