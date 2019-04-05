import cv2 as cv
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time
from autonomous_detection import AutonomousDetection

class AutonomousClassification():

    def __init__(self):
        #create a library for color determination
        colors = OrderedDict({
        	"red": (67,40,31),#255,0,0
        	"green": (24,82,51),#0,255,0
        	"blue": (25,36,63),#0,0,255
            "white": (129,142,154),#255,255,255
            "black": (0,0,0),
            "orange": (255,140,0),
            "yellow": (255,255,0),
            "purple": (128,0,128),
            "gray": (128,128,128),
            "brown": (139,69,19)})

        # allocate memory for the L*a*b* image, then initialize
        # the color names list
        self.lab_dict = np.zeros((len(colors), 1, 3), dtype="uint8")
        self.colorNames = []
        # loop over the colors dictionary
        for (i, (name, rgb)) in enumerate(colors.items()):
        	# update the L*a*b* array and the color names list
        	self.lab_dict[i] = rgb
        	self.colorNames.append(name)

        # convert the L*a*b* array from the RGB color space
        # to L*a*b*
        self.lab_dict = cv.cvtColor(self.lab_dict, cv.COLOR_RGB2LAB)
        print(self.lab_dict)
        print(self.colorNames)

        self.img_crop = []
        self.blur_crop = []
        self.canny_crop = []
        self.flood_crop = []
        self.big_contour = []
        self.white_back = []
        self.cluster = []
        self.centers = []


    def load_new_image(self, img, resize_shape, keypoints):

        self.original_image = img
        self.x_orig, self.y_orig, _ = img.shape
        self.x_resize, self.y_resize, _ = resize_shape
        self.keypoints = keypoints
        self.clear_lists()


    def crop_ROIs(self, show=False):

        for i in range(0,len(self.keypoints)):

            #find the keypoint in the resized picture
            x = int(self.keypoints[i].pt[0])
            y = int(self.keypoints[i].pt[1])

            #find the keypoint in the original picture
            x_new = int(self.x_orig*x/self.x_resize)
            y_new = int(self.y_orig*y/self.y_resize)

            #crop the potential target from the orignal image
            crop = img[y_new-30:y_new+40, x_new-30:x_new+40]
            self.img_crop.append(crop)

            if show:
                cv.imshow('Cropped Image %i' % (i), self.img_crop[i])

    def get_mask(self, show=False):

        for i in range(0, len(self.img_crop)):

            self.blur_crop.append(cv.GaussianBlur(self.img_crop[i], (5, 5), 0))
            self.blur_crop[i] = cv.pyrMeanShiftFiltering(self.blur_crop[i], 30, 30, 3)
            #cv.imshow('Blur %i' % (i), blur_crop[i])

            #detect edges and show
            self.canny_crop.append(cv.Canny(self.blur_crop[i],10,200))
            # canny_crop[i] = cv.dilate(canny_crop[i], None, iterations=2)
            # canny_crop[i] = cv.erode(canny_crop[i], None, iterations=2)
            #cv.imshow('Canny %i' % (i), canny_crop[i])

            #fill the enclosed edges to create a mask and show
            h, w = self.canny_crop[i].shape[:2]
            mask = np.zeros((h+2, w+2), np.uint8)
            edges = self.canny_crop[i].copy()
            cv.floodFill(edges, mask, (0,0), 255)
            edges = cv.bitwise_not(edges)
            self.flood_crop.append(self.canny_crop[i] | edges)
            #cv.imshow('Flood %i' % (i), self.flood_crop[i])

            #find the contours in the filled mask
            cnts = cv.findContours(self.flood_crop[i].copy(), cv.RETR_EXTERNAL,
                cv.CHAIN_APPROX_SIMPLE)
            cnts = cnts[1]

            #if there is at least one countour,
            if len(cnts) > 0:
                #find the biggest contour
                c = max(cnts, key=cv.contourArea)
                #remake the mask with only the biggest contour and show
                self.flood_crop[i][:,:] = 0
                cv.drawContours(self.flood_crop[i], [c], 0, 255, cv.FILLED)

                if show:
                    cv.imshow('Flood %i' % (i), self.flood_crop[i])

    def get_white_back(self, show=False):

        for i in range(0, len(self.flood_crop)):

            self.white_back.append(cv.bitwise_and(self.blur_crop[i],self.blur_crop[i],mask=self.flood_crop[i]))
            self.white_back[i][np.where((self.white_back[i]==[0,0,0]).all(axis=2))] = [255,255,255]

            if show:
                cv.imshow('White Background %i' % (i), self.white_back[i])

    def color_cluster(self, show=False):

        for i in range(0,len(self.white_back)):

            #convert to lab for color identification
            self.cluster.append(cv.cvtColor(self.white_back[i], cv.COLOR_BGR2LAB))
            h,w = self.cluster[i].shape[:2]
            self.cluster[i] = self.cluster[i].reshape((self.cluster[i].shape[0] * self.cluster[i].shape[1], 3))

            #reduce to 3 colors (one will be the white background)
            clt = MiniBatchKMeans(n_clusters = 3)
            labels = clt.fit_predict(self.cluster[i])

            print(clt.cluster_centers_)

            self.cluster[i] = clt.cluster_centers_.astype("uint8")[labels]

            #eliminate white from the array of colors so only two remain
            for j in range(3):
                if clt.cluster_centers_[j,0] > 254.5 and clt.cluster_centers_[j,1] > 127.5 and clt.cluster_centers_[j,2] > 127.5:
                    self.centers = np.delete(clt.cluster_centers_, j, 0)

            #reshape image
            self.cluster[i] = self.cluster[i].reshape((h, w, 3))

            #create masks to determine how many pixels are each color. The higher number is the target color and the lower is the letter color
            c1 = cv.inRange(self.cluster[i], self.centers[0,:]-1, self.centers[0,:]+1)
            c2 = cv.inRange(self.cluster[i], self.centers[1,:]-1, self.centers[1,:]+1)
            count1 = cv.countNonZero(c1)
            count2 = cv.countNonZero(c2)
            print(count1)
            print(count2)

            if count1 < count2:
                temp = np.copy(self.centers[0,:])
                self.centers[0,:] = self.centers[1,:]
                self.centers[1,:] = temp

            print(self.centers)


            #self.cluster[i][np.where((self.cluster[i].astype(int)==self.centers[0,:].astype(int)).all(axis=2))] = [0,128,128]
            #self.cluster[i][np.where((self.cluster[i].astype(int)==self.centers[1,:].astype(int)).all(axis=2))] = [255,128,128]
            self.cluster[i] = cv.cvtColor(self.cluster[i], cv.COLOR_LAB2BGR)

            if show:
                cv.imshow('Quantized %i' % (i), self.cluster[i])

            for count in range(2):
                minDist = (np.inf, None)
                for (k, row) in enumerate(self.lab_dict):
    			    # compute the distance between the current L*a*b*
    			    # color value and the mean of the image
                    d = dist.euclidean(row[0], self.centers[count])

    			    # if the distance is smaller than the current distance,
    		     	# then update the bookkeeping variable
                    if d < minDist[0]:
                        minDist = (d, k)
                print(self.colorNames[minDist[1]])

    def classify(self, show=False):

        self.crop_ROIs(show)
        self.get_mask(show)
        self.get_white_back(show)
        self.color_cluster(show)

    def clear_lists(self):

        self.img_crop.clear()
        self.blur_crop.clear()
        self.canny_crop.clear()
        self.flood_crop.clear()
        self.big_contour.clear()
        self.white_back.clear()
        self.cluster.clear()
        #self.centers.clear()

detector = AutonomousDetection()
classify = AutonomousClassification()
img = cv.imread('../comp_target_images/target8.jpg')
keypoints = detector.detect(img,1)

classify.load_new_image(img, detector.resized_image.shape, keypoints)
classify.classify(1)
cv.waitKey(0)
cv.destroyAllWindows()

img = cv.imread('../comp_target_images/target7.jpg')
keypoints = detector.detect(img,1)

classify.load_new_image(img, detector.resized_image.shape, keypoints)
classify.classify(1)
cv.waitKey(0)
cv.destroyAllWindows()
