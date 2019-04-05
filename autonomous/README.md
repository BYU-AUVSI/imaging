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

## Dependencies

The autonomous system depends on a number of standard python packages
(opencv, sklearn, scipy). Slightly more complicated is its dependency on
[Pytorch](https://pytorch.org). Installation of pytorch and torchvision depends
on your machine. For instance if you're looking to train new nets, you will
definitely want to install it with CUDA (and run it on a machine with an NVidia
card). As you can imagine, installation with CUDA is different than a CPU-only
installation (which is all you need for running the classifier).
