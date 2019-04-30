import argparse, time, threading
import numpy as np
import cv2
from PIL import Image
from client_rest import ImagingInterface
# from classify.letter_predictor import LetterPredictor
from detect.autonomous_detection import AutonomousDetection, DetectedCrop
from classify.autonomous_classification import AutonomousClassification


class AutonomousManager():

    def __init__(self, serverHost, serverPort, detection=True, classification=True):
        """
            Initialize a top-level autonomous manager

            @param serverHost: Local ip address of the machine the imaging server is running on

            @param serverPort: Port the server is running on. Default 5000

            @param detection: Whether the detector should be run by this manager. Default: True

            @param classification: Whether the classifiers should be run by this manager. Default: False 
        """
        self._should_shutdown = False
        self.client = ImagingInterface(serverHost, serverPort, isManual=False)

        # we give the option of having a machine thats running this Manager run
        #   ONLY the detection algorithm, or ONLY the classification algorithm,
        #   or both
        self.doDetection = detection
        if detection: 
            self.detector = AutonomousDetection()
        else:
            print("Turning off detection for this autonomous process")

        self.doClassification = classification
        if classification:
            self.classifier = AutonomousClassification()
        else:
            print("Turning off classification for this autonomous process")

        if not classification and not detection:
            print("ERROR:: Cant disable both detection and classification!")
            exit(1)

    def runClassification(self):
        """
            If this autonomous manager is set todo so, run classification on an available
            cropped image, if any.
        """
        return

    def runDetection(self):
        """
            If this autonomous manager is set todo so, run detection on an available
            raw image, if any.

            One issue here is client_rest expects/returns a PIL image and the 
            detector expects/returns an opencv image (aka numpy array). So 
            this method has to deal with converting between the two 
        """
        toDetect = self.client.getNextRawImage() # returns tuple of (image, image_id)

        # if there are new raw images to process
        if toDetect is not None:
            imgToDetect = np.array(toDetect[0])[:,:,::-1]
            imgId = toDetect[1]
            results = self.detector.detect(imgToDetect)

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

        while 1:

            if not self.client.ping(): # confirm we can still connect to the server
                print("WARN:: Cannot connect to server")
                time.sleep(5)

            if self.doDetection:
                self.runDetection()

            if self.doClassification:
                self.runClassification()

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
    args = parser.parse_args()

    hostname = '127.0.0.1'
    port = '5000'

    if args.host is not None:
        hostname = args.host
    if args.port is not None:
        port = args.port
    
    auto = AutonomousManager(hostname, port, args.d, args.c)
    auto.run()

if __name__ == '__main__':
    main()