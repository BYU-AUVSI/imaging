import argparse, time, threading
import numpy as np
import cv2
from PIL import Image
from client_rest import ImagingInterface, Classification
# from classify.letter_predictor import LetterPredictor
from detect.autonomous_detection import AutonomousDetection, DetectedCrop
from classify.autonomous_classification import AutonomousClassification


class AutonomousManager():

    def __init__(self, serverHost, serverPort, detection=True, classification=True, img_start=None, submit_interval=120, show=False):
        """
            Initialize a top-level autonomous manager

            @param serverHost: Local ip address of the machine the imaging server is running on

            @param serverPort: Port the server is running on. Default 5000

            @param detection: Whether the detector should be run by this manager. Default: True

            @param classification: Whether the classifiers should be run by this manager. Default: False
        """
        print("Autonomous Startup")
        self._should_shutdown = False
        self.client = ImagingInterface(serverHost, serverPort, isDebug=False, isManual=False)

        self.submit_interval = submit_interval
        self.show = show

        # for manually specifying which image in the server to start detection on
        self.img_start = img_start
        self.img_num = img_start

        # we give the option of having a machine thats running this Manager run
        #   ONLY the detection algorithm, or ONLY the classification algorithm,
        #   or both
        self.doDetection      = detection
        self.doClassification = classification
        if detection and classification:
            self.detector = AutonomousDetection()
            self.classifier = AutonomousClassification()
        elif not detection:
            print("Turning off classification for this autonomous process")
            self.detector = AutonomousDetection()
            self.doClassification = False
            self.doDetection = True
        elif not classification:
            print("Turning off detection for this autonomous process")
            self.classifier = AutonomousClassification()
            self.doDetection      = False
            self.doClassification = True

        if not classification and not detection:
            print("ERROR:: Cant disable both detection and classification!")
            exit(1)

    def submitTargets(self):
        """
            submit all pending autonomous targets
        """
        print("Submitting all pending targets...")
        self.client.postSubmitAllTargets()

    def runClassification(self):
        """
            If this autonomous manager is set todo so, run classification on an available
            cropped image, if any.
        """
        toClassify = self.client.getNextCroppedImage()

        if toClassify is not None:
            imgToClassify = np.array(toClassify[0])[:,:,::-1]
            cropId = toClassify[1]

            cropInfo = self.client.getCroppedImageInfo(cropId)
            if cropInfo is None:
                print("Failed to get cropped image info!")
                return # couldnt get info on the cropped image? weird..

            rawInfo = self.client.getImageInfo(cropInfo.imgId)
            stateMeas = None

            if rawInfo is None:
                print("Failed to get raw image info while attempting to classify!")
            else:
                # get the state measurement closest to the raw image timestamp
                stateMeas = self.client.getStateByTs(rawInfo.time_stamp)

            classified = None
            if stateMeas is not None:
                # if we were able to get a state measurement close to our raw img timestamp use it to try and decide orientation
                classified = self.classifier.classify(imgToClassify, show=False, yaw=stateMeas.yaw)
            else:
                # attempt to classify without orientation data
                classified = self.classifier.classify(imgToClassify, show=False)

            # print("Crop #: %i" % (cropId))
            if self.show:
                to_display = self.classifier.blur_crop.copy()
                cv2.putText(to_display,str(cropId),(5,50), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1,cv2.LINE_AA)
                cv2.imshow('Crop', to_display)
                key = cv2.waitKey(1) & 0xFF

            if classified is not None:
                print("Successfully classified crop {}!".format(cropId))
                print("\tshape={},letter={},shapeClr={}letterClr={},orientation={}".format(
                    classified['shape'],
                    classified['letter'],
                    classified['shapeColor'],
                    classified['letterColor'],
                    classified['orientation']))
                # TODO: Always assuming standard target for now..
                toPost = Classification(cropId, "standard",
                    orientation=classified['orientation'],
                    shape=classified['shape'],
                    bgColor=classified['shapeColor'],
                    alpha=classified['letter'],
                    alphaColor=classified['letterColor'])
                self.client.postClass(toPost)

            else:
                print("Crop # %i rejected as false positive" % (cropId))

    def runDetection(self):
        """
            If this autonomous manager is set todo so, run detection on an available
            raw image, if any.

            One issue here is client_rest expects/returns a PIL image and the
            detector expects/returns an opencv image (aka numpy array). So
            this method has to deal with converting between the two
        """
        if self.img_start is not None:
            toDetect = self.client.getRawImage(self.img_num) #returns None if the image id doesn't exist
            if toDetect is not None:
                self.img_num += 1
        else:
            toDetect = self.client.getNextRawImage() # returns tuple of (image, image_id)

        # if there are new raw images to process
        if toDetect is not None:
            imgToDetect = np.array(toDetect[0])[:,:,::-1]
            imgId = toDetect[1]

            results = self.detector.detect(imgToDetect, 0)
            print('Img #: %i' % (imgId), ' Results: %i' % (len(results)))
            print(hasattr(self.detector,'keypoints_image'))
            if self.show:# and hasattr(self.detector, 'keypoints_image'):
                to_display = self.detector.keypoints_image.copy()
                cv2.putText(to_display,str(imgId),(10,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),1,cv2.LINE_AA)
                cv2.imshow('Detected', to_display)
                key = cv2.waitKey(1) & 0xFF

            # if the detector actually returned something that's not an empty list
            if results is not None and results:
                # then lets post each of its cropped images to the server
                for detectedTarget in results:
                    pilCrop = cv2.cvtColor(detectedTarget.crop, cv2.COLOR_BGR2RGB)
                    pilCrop = Image.fromarray(pilCrop)
                    self.client.postCroppedImage(imgId, pilCrop, detectedTarget.topLeft, detectedTarget.bottomRight)

    def run(self):
        """
        Sit and spin, checking for new images and processing them as necessary
        """

        last_submit = time.time()
        while 1:

            if not self.client.ping(): # confirm we can still connect to the server
                print("WARN:: Cannot connect to server")
                time.sleep(5)

            if self.doDetection:
                self.runDetection()

            if self.doClassification:
                self.runClassification()

                if (time.time() - last_submit) > self.submit_interval:
                    self.submitTargets()
                    last_submit = time.time()

            time.sleep(0.1)

    def shutdown(self):
        self._should_shutdown = True

def startAutonomousManagerThread(serverHost, serverPort, detection, classification):
    auto = AutonomousManager(serverHost, serverPort, detection, classification)
    auto.run()

def main():
    parser = argparse.ArgumentParser(description="Autonomous Manager Help")
    parser.add_argument('-H', '--host', metavar='hostname', help='The hostname IP address of where the server is. Default: 127.0.0.1')
    parser.add_argument('-P', '--port', metavar='port', help='The hostname port for the server. Default: 5000')
    parser.add_argument("-d", action='store_false', help="Detector only - only run the detector in this process")
    parser.add_argument("-c", action='store_false', help="Classifier only - only run the classifier in the process")
    parser.add_argument('-i', '--img', metavar='img_start', type=int, help="Manually specify image number for detection to start on")
    parser.add_argument('-s', '--submit_interval', metavar='interval', type=int, help="If classifier is turned on, how often autonomously classified targets should be submitted to the judges, in seconds.")
    parser.add_argument('--show', action='store_true', help="Turn on to show intermediate image state and debug logging")
    args = parser.parse_args()

    hostname = '127.0.0.1'
    port = '5000'
    img_start = None

    print("{} {}".format(args.d, args.c))

    if args.host is not None:
        hostname = args.host
    if args.port is not None:
        port = args.port
    if args.img is not None:
        img_start = args.img

    interval = 120
    if args.submit_interval is not None and args.submit_interval > 0:
        interval = args.submit_interval

    auto = AutonomousManager(hostname, port, args.d, args.c, img_start, interval, args.show)
    auto.run()

if __name__ == '__main__':
    main()
