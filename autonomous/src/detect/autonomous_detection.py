import cv2 as cv
import numpy as np
import imutils

class DetectedCrop():
    """
    Holds information on an autonomously cropped image. Namely the cropped image itself,
    and the coordinates where the crop took place relative to the original raw image
    """

    def __init__(self, cropped, topLeft, bottomRight):
        self.crop        = cropped
        self.topLeft     = topLeft
        self.bottomRight = bottomRight

#Pass in cv image
class AutonomousDetection():

    def __init__(self):

        #set the parameters for blob detection
        params = cv.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 200
        #filter by Area
        params.filterByColor = False
        params.filterByArea = True
        params.minArea = 50#200
        params.maxArea = 1000#2500
        #filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.7
        #filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5
        #filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.1
        params.maxInertiaRatio = 1.0

        #create the blob detector and detect significant blobs
        self.blob_detector = cv.SimpleBlobDetector_create(params)

        self.img_crop = []

    def detect(self, img, show=False):
        """
            Given a raw image, run detection algorithm to try and find targets.

            @type img: Opencv image
            @param img: An original uncropped image as an opencv image. Detection 
                will be run against this image

            @type show: boolean
            @param show: Specify if you want to show the output of detection in a window.
                Should be turned off in production

            @rtype: list of opencv images
            @returns: A list of cropped regions of interest (ROIs) from the original image
        """
        self.original_image = img
        self.resized_image = imutils.resize(img, width=600) #bassler images
        #self.resized_image = imutils.resize(img, width=1200) #sony images

        self.y_orig, self.x_orig, _ = img.shape
        self.y_resize, self.x_resize, _ = self.resized_image.shape

        self.img_crop.clear()

        self.preprocess()
        self.detect_ROIs(500)
        self.crop_ROIs()


        if show:
            self.keypoints_image = cv.drawKeypoints(self.resized_image, self.keypoints,
                np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv.imshow('Resized', self.resized_image)
            cv.imshow('Preprocess', self.preprocessed)
            cv.imshow('Canny', self.canny)
            cv.imshow('Flood', self.flood)
            cv.imshow('Detection', self.keypoints_image)
            for i in range(len(self.img_crop)):
                cv.imshow('Cropped Image %i' % (i), self.img_crop[i].crop)
            # cv.waitKey(0)
            # cv.destroyAllWindows()

        return self.img_crop

    def preprocess(self):
        img_blur = cv.GaussianBlur(self.resized_image, (5, 5), 0)
        #self.preprocessed = cv.pyrMeanShiftFiltering(self.resized_image, 10, 10, 3) #bassler images
        self.preprocessed = cv.pyrMeanShiftFiltering(img_blur, 30, 30, 3) #sony images
        #self.preprocessed = cv.bilateralFilter(img_blur, 10, 150, 150) #faster bilateral filter

    def detect_ROIs(self, edge_limit=500):
        self.canny = cv.Canny(self.preprocessed, 10, edge_limit)
        self.canny = cv.dilate(self.canny, None, iterations=1)
        self.canny = cv.erode(self.canny, None, iterations=1)
        #self.floodfill()
        self.flood = self.canny
        self.keypoints = self.blob_detector.detect(self.flood)
        self.reduce_keypoints()

    def floodfill(self):
        h, w = self.canny.shape[:2]
        self.canny[0,:] = 0
        self.canny[h-1,:] = 0
        self.canny[:,0] = 0
        self.canny[:,w-1] = 0
        mask = np.zeros((h+2, w+2), np.uint8)
        edges = self.canny.copy()
        cv.floodFill(edges, mask, (0,0), 255)
        edges = cv.bitwise_not(edges)
        self.flood = self.canny | edges

    #check to see if keypoints are too close to each other
    def reduce_keypoints(self):
        to_delete = []
        for i in range(len(self.keypoints)):
            if i not in to_delete:
                for j in range(len(self.keypoints)):
                    if i != j and j not in to_delete:
                        dist = ((self.keypoints[j].pt[0]-self.keypoints[i].pt[0])**2+(self.keypoints[j].pt[1]-self.keypoints[i].pt[1])**2)**(1/2)
                        if dist < 100:#100
                            to_delete.append(j)
        for n in to_delete:
            self.keypoints[n] = None

    def crop_ROIs(self):

        for i in range(len(self.keypoints)):

            if self.keypoints[i] is not None:
                #find the keypoint in the resized picture
                x = int(self.keypoints[i].pt[0])
                y = int(self.keypoints[i].pt[1])

                #find the keypoint in the original picture
                x_new = int(self.x_orig*x/self.x_resize)
                y_new = int(self.y_orig*y/self.y_resize)

                #crop the potential target from the orignal image
                #crop = img[y_new-30:y_new+40, x_new-30:x_new+40]

                topY    = max(1,y_new-90)
                bottomY = min(y_new+120,self.y_orig)
                leftX   = max(1,x_new-90)
                rightX  = min(x_new+120,self.x_orig)

                crop = self.original_image[topY:bottomY, leftX:rightX] #sony image

                detected = DetectedCrop(crop, (topY, leftX), (bottomY, rightX))
                self.img_crop.append(detected)
