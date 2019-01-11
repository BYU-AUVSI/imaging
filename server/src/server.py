from flask import Flask
from apis import api
import argparse


def main():
    parser = argparse.ArgumentParser(description="Imaging Server Help")
    parser.add_argument('-H', '--host', metavar='hostname', help='The hostname to publish the server on. For Docker, this should be 0.0.0.0')

    args = parser.parse_args()

    hostname = "127.0.0.1"
    if args.host is not None:
        hostname = args.host

    app = Flask(__name__)
    api.init_app(app)

    app.run(debug=True, host=hostname)


if __name__ == '__main__':
    main()
