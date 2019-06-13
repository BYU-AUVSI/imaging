from detect.autonomous_detection import AutonomousDetection
from classify.autonomous_classification import AutonomousClassification
import cv2 as cv
import time
from os import listdir


def main():
    detector = AutonomousDetection()
    classifier = AutonomousClassification()
    pic_names = listdir('chatico_raw')

    count = 0

    for i in range(len(pic_names)):
    #for i in range(1):
        img = cv.imread('chatico_raw/' + pic_names[i])
        #img = cv.imread('../dji-targets-Mar20/DJI_0004.JPG')
        start = time.time()
        crops = detector.detect(img, 1)
        print("Detector Time Elapsed: %f" % (time.time()-start))
        print("ROIs detected: %i" % (len(crops)))
        for j in range(len(crops)):
            res = classifier.classify(crops[j].crop, 1)

            if res is None:
                print('Not a target\n')

            else:
                print('Target!\n')
                print(res)
                print('\n')

        # for j in range(len(classifier.cluster)):
        #     if classifier.cluster[j] is not None:
        #         cv.imwrite('letters/letter%i.jpg' % (count), classifier.cluster[j])
        #         count += 1

            cv.waitKey(0)
            cv.destroyAllWindows()



if __name__ == "__main__":
    main()
