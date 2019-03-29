# Autonomous Classification

This is the source code for the autonomous detector/classifier client. Like manual
classification, the autonomous client does not need to be run on the server, but 
must be able to connect to it.

**TODO: INSERT IMAGE SHOWING FLOWCHART OVERVIEW OF HOW THIS WORKS**

## Structure

`util` Hold a lot of useful scripts used to help train and test autonomous
 classifier/detector. This includes dataset generators for the letter and shape
 classifier. Pytorch scripts for training both of them, and scripts for running
 test images against the nets.

`src` Actual source code that's run for the autonomous system.