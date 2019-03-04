from flask import Flask, json
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
        super(ServerThread, self).__init__()
        self.srv = make_server(hostname, 5000, app)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        print("Start server")
        # keep the server up until we specifically tell it to die
        self.srv.serve_forever()

    def shutdown(self):
        self.srv.shutdown()


def signal_handler(sig, frame):
    # shut the all our server processes down safely
    global server
    print('Shutting Down')
    server.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Imaging Server Help")
    parser.add_argument('-H', '--host', metavar='hostname', help='The hostname to publish the server on. For Docker, this should be 0.0.0.0')

    args = parser.parse_args()

    hostname = "0.0.0.0"
    if args.host is not None:
        hostname = args.host
    
    app = Flask(__name__)
    api.init_app(app)
    #app.start(debug=True, host=hostname)

    global server
    server = ServerThread(app, hostname)

    # now that our server thread is setup, we can configure sigint to properly close it
    signal.signal(signal.SIGINT, signal_handler)

    server.start()
    print("Server Started!")
        

if __name__ == '__main__':
    main()
