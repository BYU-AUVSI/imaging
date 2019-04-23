import cv2 as cv
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time
from detect.autonomous_detection import AutonomousDetection
from letter_predictor import LetterPredictor
from shape_predictor import ShapePredictor


#TODO: Add orientation function
#      Pack into dict for submission

class AutonomousClassification():

    def __init__(self):
        #create a library for color determination in RGB
        colors = OrderedDict({
        	"red": (215,150,150),#255,0,0
        	"green": (100,255,200),#0,255,0
        	"blue": (80,200,200),#0,0,255
            "white": (255,255,255),#255,255,255
            "black": (0,0,0),
            "orange": (250,200,100),
            "yellow": (250,250,190),
            "purple": (170,165,250),
            "gray": (200,200,200),
            "brown": (220,210,220)})

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

        self.shape_classifier = ShapePredictor()
        self.letter_classifier = LetterPredictor()


    def classify(self, cropped_img, show=False):

        try:
            self.img_crop = imutils.resize(cropped_img, width=200)
            self.get_mask(show)
            self.get_black_back(show)
            self.color_cluster(show)
            self.get_colors()
            self.get_shape()
            self.get_letter()
            #self.get_orientation()
            self.orientation = 'N'

            if self.letter_contour is None:
                return None

            print(self.colors)
            print(self.shape)
            print(self.letter)

        except Exception:
            return None
            print("Classifier Exception!")


    def get_mask(self, show=False):

        self.blur_crop = cv.GaussianBlur(self.img_crop, (5, 5), 0)     #**IS THIS NECESSARY?**
        self.blur_crop = cv.pyrMeanShiftFiltering(self.blur_crop, 30, 30, 3)

        if show:
            cv.imshow('Blur', self.blur_crop)

        #detect edges and show
        self.canny_crop = cv.Canny(self.blur_crop,10,300)
        #dilating the edges often closes edges that were originally not connected
        self.canny_crop = cv.dilate(self.canny_crop, None, iterations=1)
        #self.canny_crop[i] = cv.erode(self.canny_crop[i], None, iterations=1)
        if show:
            cv.imshow('Canny', self.canny_crop)

        #fill the enclosed edges to create a mask and show
        h, w = self.canny_crop.shape[:2]
        self.canny_crop[0,:] = 0
        self.canny_crop[h-1,:] = 0
        self.canny_crop[:,0] = 0
        self.canny_crop[:,w-1] = 0
        mask = np.zeros((h+2, w+2), np.uint8)
        edges = self.canny_crop.copy()
        cv.floodFill(edges, mask, (0,0), 255)
        edges = cv.bitwise_not(edges)
        self.flood_crop = self.canny_crop | edges
        self.flood_crop = cv.erode(self.flood_crop, None, iterations=2)

        #find the contours in the filled mask
        cnts = cv.findContours(self.flood_crop.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[1]

        #if there is at least one countour,
        if len(cnts) > 0:
            #find the biggest contour
            self.c = max(cnts, key=cv.contourArea)
            #remake the mask with only the biggest contour and show
            self.flood_crop[:,:] = 0

            cv.drawContours(self.flood_crop, [self.c], 0, 255, cv.FILLED)

            shape_img = cv.cvtColor(self.flood_crop, cv.COLOR_GRAY2BGR)
            self.shape = self.shape_classifier.predict(shape_img)

            print('Shape Guess: %s' % (self.shape))
            #erode the mask to eliminate any leftover background
            #self.flood_crop[i] = cv.erode(self.flood_crop[i], None, iterations=2)

            # M = cv.moments(self.c)
            # cX = int(M["m10"] / M["m00"])
            # cY = int(M["m01"] / M["m00"])
            #
            # self.shape_crop = self.img_crop.copy()[max(1,cY-70):cY+70, max(1,cX-70):cX+70]
            #self.blur_crop = self.blur_crop[max(1,cY-70):cY+70, max(1,cX-70):cX+70]
            #self.flood_crop = self.flood_crop[max(1,cY-70):cY+70, max(1,cX-70):cX+70]

            if show:
                cv.imshow('Flood', self.flood_crop)

        else:
            #print('Flood %i failed: no significant contours in crop' % (i))
            self.flood_crop = None
            self.c = None


    def get_black_back(self, show=False):
        #NOTE: Changing color to black
        if self.flood_crop is not None:
            self.black_back = cv.bitwise_and(self.blur_crop,self.blur_crop,mask=self.flood_crop)
            #self.white_back[i][np.where((self.white_back[i]==[0,0,0]).all(axis=2))] = [255,255,255]

            if show:
                cv.imshow('Black Background', self.black_back)

        else:
            self.black_back = None


    def color_cluster(self, show=False):

        if self.black_back is not None:
            #convert to lab for color identification
            self.cluster = cv.cvtColor(self.black_back, cv.COLOR_BGR2LAB)
            h,w = self.cluster.shape[:2]
            self.cluster = self.cluster.reshape((self.cluster.shape[0] * self.cluster.shape[1], 3))

            #reduce to 3 colors (one will be the white background)
            clt = MiniBatchKMeans(n_clusters = 3, reassignment_ratio=0.2)
            labels = clt.fit_predict(self.cluster)

            self.cluster = clt.cluster_centers_.astype("uint8")[labels]

            #print(clt.cluster_centers_)
            #eliminate white from the array of colors so only two remain
            #NOTE: now looks for black
            for j in range(3):
                if clt.cluster_centers_[j,0] < 2.0 and 127.0 < clt.cluster_centers_[j,1] < 129.0 and 127.0 < clt.cluster_centers_[j,2] < 129.0:
                    self.centers = np.delete(clt.cluster_centers_, j, 0)
                    break

            #reshape image
            self.cluster = self.cluster.reshape((h, w, 3))

            #create masks to determine how many pixels are each color. The higher number is the target color and the lower is the letter color
            c1 = cv.inRange(self.cluster, self.centers[0,:]-1, self.centers[0,:]+1)
            c2 = cv.inRange(self.cluster, self.centers[1,:]-1, self.centers[1,:]+1)
            count1 = cv.countNonZero(c1)
            count2 = cv.countNonZero(c2)

            if count1 < count2:
                temp = np.copy(self.centers[0,:])
                self.centers[0,:] = self.centers[1,:]
                self.centers[1,:] = temp

            #print(self.centers)
            #Reject if one of the colors is black
            for j in range(2):
                if self.centers[j,0] < 2.0 and 127.0 < self.centers[j,1] < 129.0 and 127.0 < self.centers[j,2] < 129.0:
                    #print('Cluster %i failed: Not enough colors identified' % (i))
                    self.cluster = None
                    self.letter_contour = None
                    break

            d_centers = dist.euclidean(self.centers[0], self.centers[1])
            if d_centers < 25:#15
                self.cluster = None
                self.letter_contour = None
                #print('Cluster %i failed: Clustered too close' % (i))


            #NOTE: now changes letter to white and background to black
            if self.cluster is not None:
                self.cluster[np.where((self.cluster.astype(int)==self.centers[0,:].astype(int)).all(axis=2))] = [0,128,128]
                self.cluster[np.where((self.cluster.astype(int)==self.centers[1,:].astype(int)).all(axis=2))] = [255,128,128]
                self.cluster = cv.cvtColor(self.cluster, cv.COLOR_LAB2BGR)

                self.cluster = cv.cvtColor(self.cluster, cv.COLOR_BGR2GRAY)
                cnts = cv.findContours(self.cluster.copy(), cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
                contours = cnts[1]
                hierarchy = cnts[2]

                #if there is at least one countour,
                if len(contours) > 0:
                    #find the biggest contour
                    areas = [cv.contourArea(c) for c in contours] # get the area of each contour
                    max_index = np.argmax(areas)
                    c = max(contours, key=cv.contourArea)
                    #remake the mask with only the biggest contour and show
                    self.cluster[:,:] = 0

                    cv.drawContours(self.cluster, [c], 0, 255, cv.FILLED)
                    child = hierarchy[0][max_index][2]
                    if child != -1:
                        cv.drawContours(self.cluster, [contours[child]], 0, 0, cv.FILLED)

                    self.cluster = cv.bitwise_not(self.cluster)

                    #Two false positive checks:
                    #   If a large portion of the edge of the "letter" is on the border
                    #   If the "letter" isn't big enough related to the shape
                    letter_edge = cv.Canny(self.cluster, 10, 50)
                    letter_edge_count = cv.countNonZero(letter_edge)
                    shape_edge = cv.Canny(self.flood_crop, 10, 50)
                    combined_edge = cv.bitwise_and(letter_edge, shape_edge)
                    combined_edge_count = cv.countNonZero(combined_edge)

                    h,w = self.cluster.shape
                    letter_area = h*w - cv.countNonZero(self.cluster)
                    shape_area = cv.countNonZero(self.flood_crop)

                    if combined_edge_count/letter_edge_count > 0.1 or letter_area/shape_area < 0.05:
                        #print("Cluster %i failed: Letter not likely" % (i))
                        self.cluster = None


                    self.letter_contour = self.cluster.copy()


                if show and self.letter_contour is not None:
                    cv.imshow('Letter Contour', self.letter_contour)

                #Deterimine colors
                # for count in range(2):
                #     minDist = (np.inf, None)
                #     for (k, row) in enumerate(self.lab_dict):
    	        #         # compute the distance between the current L*a*b*
    			#         # color value and the mean of the image
                #         d = dist.euclidean(row[0], self.centers[count])
                #
    			#         # if the distance is smaller than the current distance,
    		    #  	    # then update the bookkeeping variable
                #         if d < minDist[0]:
                #             minDist = (d, k)
                #     print('Color %i: %s' % (count,self.colorNames[minDist[1]]))

        else:
            self.cluster = None
            self.letter_contour = None


    def get_colors(self):

        self.colors = []

        if self.letter_contour is not None:

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

                self.colors.append(self.colorNames[minDist[1]])

        else:
            self.colors = None


    def get_shape(self):

        if self.flood_crop is not None:
            # peri = cv.arcLength(self.c, True)
            # approx = cv.approxPolyDP(self.c, 0.02 * peri, True)
            # #print('Shape %i: %i' % (i,len(approx)))

            shape_img = cv.cvtColor(self.flood_crop, cv.COLOR_GRAY2BGR)
            self.shape = self.shape_classifier.predict(shape_img)

        else:
            self.shape = None

    def get_letter(self):

        if self.letter_contour is not None:

            letter_img = cv.cvtColor(self.letter_contour, cv.COLOR_GRAY2BGR)
            self.letter = self.letter_classifier.predict(letter_img)

        else:
            self.letter = None
