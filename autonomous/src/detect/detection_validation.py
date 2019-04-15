from autonomous_detection import AutonomousDetection
from autonomous_classification import AutonomousClassification
import cv2 as cv
import time
from os import listdir


def main():
    detector = AutonomousDetection()
    classifier = AutonomousClassification()
    pic_names = listdir('../dji-targets-Mar20')

    count = 0

    #for i in range(len(pic_names)):
    for i in range(1):
        #img = cv.imread('../dji-targets-Mar20/' + pic_names[i])
        img = cv.imread('../dji-targets-Mar20/DJI_0004.JPG')
        start = time.time()
        crops = detector.detect(img, 1)
        print("Detector Time Elapsed: %f" % (time.time()-start))
        print("ROIs detected: %i" % (len(crops)))
        classifier.load_new_crops(crops)
        classifier.classify(1)
        for j in range(len(classifier.cluster)):
            if classifier.cluster[j] is not None:
                cv.imwrite('letters/letter%i.jpg' % (count), classifier.cluster[j])
                count += 1
        crops = []
        cv.waitKey(0)
        cv.destroyAllWindows()



if __name__ == "__main__":
    main()
