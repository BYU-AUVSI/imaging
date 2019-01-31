# Server

The is the code to run the server. We should fill this readme out more.

![server overview](docs/img/serverFlowchart.png)

The server has 3 main jobs:

1. Ingest all new data from the ROS network into the database.
2. Service requests from the manual and autonomous classification clients. Keeping track of intermediate state and allowing multiple clients to run simultaneously.
3. Submit completed data to the interop relay/server (TODO: idk if interop will be integrated with this yet)

## Structure

`conf/` Holds all configurable parameters for the server. Make sure to set these up properly

`docs/` Documentation stuff to help describe what the heck is happening

`setup/` Contains scripts and helper files to setup the server on a fresh machine.

`src/` All source code for the server

Any folder called `internal` is stuff you dont need to worry about, unless you're really digging into things.

## Running It

Once installed (see [installation](#installation) guide below), running the rest server is easy. From this folder:

`python src/server.py`

If you also want to run the ros ingestion code, place this repository in your workspace/src folder. once it's built, you can run the ingester with:

`rosrun imaging_ros_ingester ros_ingest`

(Notice that the server and ros_ingester and completely independent of each other. This is intentional)

## REST API

All REST API documentation can be found on the homepage of server, once you start it running (with `python src/server.py`). If the server is running on your machine for example, you can see the documentation at: http://localhost:5000

## Installation

There are two main installation methods: production and development.

### Development

If you're looking to develop and actively test/use the server code on your own machine, run the `./setup/dev_setup.sh` script. It's highly recommended (especially if you're using ubuntu 16.04), to use a [conda environment](https://conda.io/docs/user-guide/install/index.html).

### Production

TODO: talk about docker img

## Motivation

The server/client setup minimizes imaging's dependence on ROS, thus increasing transferability. Want to use something other than ROS in the future? Then all you need to do is change the < 100 line ros_handler.py script, (which describes how to pull data in from the plane and publish it out to interop) and the rest (hehe) of the server-client codebase is good to go!
