import cv2 as cv
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time

#Pass in cv image
class AutonomousDetection():

    def __init__(self):

        #set the parameters for blob detection
        params = cv.SimpleBlobDetector_Params()
        params.minThreshold = 0;
        params.maxThreshold = 200;
        #filter by Area
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 200
        #filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.7
        #filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.0
        #filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.3

        #create the blob detector and detect significant blobs
        self.blob_detector = cv.SimpleBlobDetector_create(params)


    def detect(self, img, show=0):
        self.original_image = img
        self.resized_image = imutils.resize(img, width=600)

        self.preprocess()
        self.detectROI()

        if show:
            self.keypoints_image = cv.drawKeypoints(self.resized_image, self.keypoints,
                np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv.imshow('Detection', self.keypoints_image)
            cv.waitKey(0)
            cv.destroyAllWindows()

        return self.keypoints

    def preprocess(self):
        img_blur = cv.GaussianBlur(self.resized_image, (5, 5), 0)
        self.preprocessed = cv.pyrMeanShiftFiltering(img_blur, 10, 10, 3) #bassler images
        #self.preprocessed = cv.pyrMeanShiftFiltering(img_blur, 30, 30, 3) #sony images

    def detectROI(self, edge_limit=500):
        self.canny = cv.Canny(self.preprocessed, 10, edge_limit)
        self.keypoints = self.blob_detector.detect(self.canny)


# detector = AutonomousDetection()
# img = cv.imread('../comp_target_images/target7.jpg')
# detector.detect(img,1)
#
# img = cv.imread('../comp_target_images/target8.jpg')
# detector.detect(img,1)
