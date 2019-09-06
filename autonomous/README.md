# Autonomous Classification

This is the source code for the autonomous detector/classifier client. Like manual
classification, the autonomous client does not need to be run on the server, but
must be able to connect to it. In the competition, we had it run on a single laptop.

## Structure

`util` Hold a lot of useful scripts used to help train and test the autonomous
 classifier/detector. This includes dataset generators for the letter and shape
 CNN classifiers. Pytorch scripts for training both of them, and scripts for running
 test images against the nets.

`src` Actual source code that's run for the autonomous system.

## Running it

From this folder you can run autonomous with this:

`python src/autonomous.py`

A number of command line options are provided to customize how the system is run.
You can view these options through a standard help call:

`python src/autonomous.py --help`

Of note are the `-d` and `-c` arguments which allow you to decouple the detector
and classifier and run them on separate machines. Note: everything is built in
such a way that you can run multiple autonomous clients without issue (ie: they
will not run detection or classification on the same images).

A typical command for running autonomous in a competition setting would be something like this:

`python src/autonomous.py --host 192.168.1.10 --port 5000 --show`

## Architecture

Here are some diagrams to try and help explain how the crap this all works:

![autonomous flow chart overview](docs/img/autonomous_classification.png)

![autonomous process example](docs/img/autonomous_vision.png)

You'll notice the source code is split into two major pieces: detect and classify.

Detection relies on a variety of opencv methods and works fairly well. The main
challenge with detection is finding a balance between rejecting false positives,
and recognizing actual targets. It may be worth looking into supplementing this
step with a neural net.

Classification uses a variety of methods to attempt and classify cropped targets
created by the detector. The main challenge here is getting large and _diverse_
datasets to adequately train the nets. Also, orientation classification could use
a bit of love.

## Dependencies

The autonomous system depends on a number of standard python packages
(opencv, sklearn, scipy). Slightly more complicated is its dependency on
[Pytorch](https://pytorch.org). Installation of pytorch and torchvision depends
on your machine. For instance if you're looking to train new nets, you will
definitely want to install it with CUDA (and run it on a machine with an NVidia
card). As you can imagine, installation with CUDA is different than a CPU-only
installation (which is all you need for running the classifier).

## Known Shortcomings

Miraculously, we were the only team to submit an autonomously detected target last year, 
even though we just copied what other teams did. It is by no means perfect, and
needs a lot of work. Here are a few of the known shortcomings, and some things that
can be done to overcome them.

First, the detection algorithm is FREAKING slow! It takes around 3 seconds to run
the pyrmeanshiftfilter algorithm on a CPU, which is no bueno. To overcome this,
we built functionality to run multiple detectors and classifiers at the same time
so that the system can keep up with the camera (there is a problem with running 
multiple detectors too, which I'll explain). We hightly recommend that you find
a faster algorithm that works just as well. We experimented with several detectors
but couldn't find anything that worked as well. Also, try getting this to run on
a GPU using CUDA (you may need to switch the system to C++ for this). I don't think
there is a pyrmeanshiftfilter CUDA implementation in OpenCV, so you may need to 
write your own, or you can experiment with other CUDA algorithms.

Which brings me to the next point. Today, traditional detection algorithms use deep
learning. If you don't know what that is or how it works, read up about it, a lot! 
Switching the whole autonomous system to run on CNNs would be ideal, but you'll 
need to collect a ton of data to get it to work. We tried to keep a lot of target
and no target images for this, but it won't be enough. If you want to get a detector
working with a CNN, you'll need to generate realistic looking synthetic data on real
image field backgrounds. This will take a lot of time. You should look into using 
generative adversarial networks (GANs) to do this, altough I'm not totally sure
how that would work. 

For determining the orientation of a letter, you'll want to add an output to the letter
classifier that gives it a continuous orientation, label all of the training data with
orientation angles, and retrain. The way we tried to do orientation classification
sucked and didn't work at all. Also, try to do color classification with deep learning.
We spent a little bit of time trying this but couldn't get it to work very well.

Running multiple detectors and classifiers is really nice, but there is one issue with
it. Right before the competition, we made it so you can tell the server which image you
want to start on. This is nice because if it gets behind, you can tell it to start farther
up, or if the plane is flying over an area where there are no targets, you can stop it 
and restart it where you want. However, when you do this, the server will just give you
new images sequentially without checking whether the image has already been examined by
a different autonomous client, so you can't run multiple detectors at once when you do
this. A slight modification to the server will need to be made to compensate for this. 
