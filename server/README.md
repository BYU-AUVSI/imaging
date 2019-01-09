# Server

The is the code to run the server. We should fill this readme out more.

**Roadmap- ros_ingester / DAO -> REST API -> dockerize**

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

## REST API

All API documentation can be found on the root of the website. (ie: http://localhost:5000 if running on your machine)

## Installation

TODO: easy installation with docker, or the dev_setup.sh script

## Motivation

The server/client setup minimizes imaging's dependence on ROS, thus increasing transferability. Want to use something other than ROS in the future? Then all you need to do is change the < 100 line ros_ingest.py script, (which describes how to pull data in from the plane) and you're good to go!
