import argparse
from client_rest import ImagingInterface
from classify.letter_predictor import LetterPredictor
from detect.autonomous_detection import AutonomousDetection
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
        
        self.client = ImagingInterface(serverHost, serverPort, isManual=False)

        self.doDetection = detection
        if detection:
            self.detector = AutonomousDetection()

        self.doClassification = classification
        if classification:
            self.classifier = AutonomousClassification()


    def run(self):
        """
        Sit and spin, checking for new images
        """
        print("heyyyy")


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
    
    
    auto = AutonomousManager(hostname, port)
    auto.run()

if __name__ == '__main__':
    main()