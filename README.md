# Imaging [![Build Status](https://travis-ci.com/BYU-AUVSI/imaging.svg)](https://travis-ci.com/BYU-AUVSI/imaging)

This package contains (most) all the code used by the imaging subsystem.

`server/` Contains code that is run on the ground station server

`client/` Client code runs the manual classification client and autonomous monitor. This code is run on personal machines.

`autonomous` Code used by the autonomous classifier

**What isn't included?**

The driver for the camera, currently the [Sony a6000](https://github.com/BYU-AUVSI/a6000_ros.git) is in its own repository. This is because it is the only piece of code that needs to be run onboard the actual plane, thus we want to make it small and easy to clone onto the planes workspace.

## Quick Start

Want to quickly see the imaging code in action? We have a convenient demo for this very purpose. With docker installed on your machine run:

`./server/setup/start-demo.sh`

This will install a barebones docker image on your machine that reads in a demo data set so that the server is populated with good test data. Note for convenience, this image does not use any of the server's ROS components, making it _unsuitable_ for running in a production environment.

## Dependencies

This package relies on a few other BYU-Auvsi packages. Changes to them could have effects here.

- [BYU-AUVSI/rosplane/rosplane_msgs/msg/State.msg](https://github.com/BYU-AUVSI/rosplane/blob/RC1.0/rosplane_msgs/msg/State.msg)

- [BYU-AUVSI/inertial_sense_ros/GPS/GPS.msg](https://github.com/BYU-AUVSI/inertial_sense_ros)