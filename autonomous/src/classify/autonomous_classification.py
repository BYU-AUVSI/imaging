import cv2 as cv
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time
from detect.autonomous_detection import AutonomousDetection

#TODO: Methods for rejecting false positives to implement:
#       Compare how much of edge of "letter" matches with edge of
#           shape. If too high, reject
#       Compare ratio of size of letter to size of shape. If too low, reject

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

        self.img_crop = []
        self.blur_crop = []
        self.canny_crop = []
        self.flood_crop = []
        self.c = []
        self.big_contour = []
        self.white_back = []
        self.cluster = []
        self.centers = []


    def load_new_crops(self, img_crop):

        self.clear_lists()
        for i in range(len(img_crop)):
            self.img_crop.append(imutils.resize(img_crop[i], width=200))


    def get_mask(self, show=False):

        for i in range(len(self.img_crop)):

            self.blur_crop.append(cv.GaussianBlur(self.img_crop[i], (5, 5), 0))     #**IS THIS NECESSARY?**
            self.blur_crop[i] = cv.pyrMeanShiftFiltering(self.blur_crop[i], 30, 30, 3)
            cv.imshow('Blur %i' % (i), self.blur_crop[i])

            #detect edges and show
            self.canny_crop.append(cv.Canny(self.blur_crop[i],10,200))
            #dilating the edges often closes edges that were originally not connected
            self.canny_crop[i] = cv.dilate(self.canny_crop[i], None, iterations=1)
            #self.canny_crop[i] = cv.erode(self.canny_crop[i], None, iterations=1)
            cv.imshow('Canny %i' % (i), self.canny_crop[i])

            #fill the enclosed edges to create a mask and show
            h, w = self.canny_crop[i].shape[:2]
            self.canny_crop[i][0,:] = 0
            self.canny_crop[i][h-1,:] = 0
            self.canny_crop[i][:,0] = 0
            self.canny_crop[i][:,w-1] = 0
            mask = np.zeros((h+2, w+2), np.uint8)
            edges = self.canny_crop[i].copy()
            cv.floodFill(edges, mask, (0,0), 255)
            edges = cv.bitwise_not(edges)
            self.flood_crop.append(self.canny_crop[i] | edges)
            self.flood_crop[i] = cv.erode(self.flood_crop[i], None, iterations=2)

            #find the contours in the filled mask
            cnts = cv.findContours(self.flood_crop[i].copy(), cv.RETR_EXTERNAL,
                cv.CHAIN_APPROX_SIMPLE)
            cnts = cnts[1]

            #if there is at least one countour,
            if len(cnts) > 0:
                #find the biggest contour
                self.c.append(max(cnts, key=cv.contourArea))
                #remake the mask with only the biggest contour and show
                self.flood_crop[i][:,:] = 0

                cv.drawContours(self.flood_crop[i], [self.c[i]], 0, 255, cv.FILLED)
                #erode the mask to eliminate any leftover background
                #self.flood_crop[i] = cv.erode(self.flood_crop[i], None, iterations=2)

                # M = cv.moments(self.c[i])
                # cX = int(M["m10"] / M["m00"])
                # cY = int(M["m01"] / M["m00"])
                # self.blur_crop[i] = self.blur_crop[i][max(1,cY-70):cY+70, max(1,cX-70):cX+70]
                # self.flood_crop[i] = self.flood_crop[i][max(1,cY-70):cY+70, max(1,cX-70):cX+70]

                if show:
                    cv.imshow('Flood %i' % (i), self.flood_crop[i])

            else:
                #print('Flood %i failed: no significant contours in crop' % (i))
                self.flood_crop[i] = None
                self.c.append(None)

    def get_white_back(self, show=False):
        #NOTE: Changing color to black
        for i in range(0, len(self.flood_crop)):
            if self.flood_crop[i] is not None:
                self.white_back.append(cv.bitwise_and(self.blur_crop[i],self.blur_crop[i],mask=self.flood_crop[i]))
                #self.white_back[i][np.where((self.white_back[i]==[0,0,0]).all(axis=2))] = [255,255,255]

                if show:
                    cv.imshow('White Background %i' % (i), self.white_back[i])

            else:
                self.white_back.append(None)

    def color_cluster(self, show=False):

        for i in range(0,len(self.white_back)):
            if self.white_back[i] is not None:
                #convert to lab for color identification
                self.cluster.append(cv.cvtColor(self.white_back[i], cv.COLOR_BGR2LAB))
                h,w = self.cluster[i].shape[:2]
                self.cluster[i] = self.cluster[i].reshape((self.cluster[i].shape[0] * self.cluster[i].shape[1], 3))

                #reduce to 3 colors (one will be the white background)
                clt = MiniBatchKMeans(n_clusters = 3, reassignment_ratio=0.2)
                labels = clt.fit_predict(self.cluster[i])

                self.cluster[i] = clt.cluster_centers_.astype("uint8")[labels]

                #print(clt.cluster_centers_)
                #eliminate white from the array of colors so only two remain
                #NOTE: now looks for black
                for j in range(3):
                    if clt.cluster_centers_[j,0] < 2.0 and 127.0 < clt.cluster_centers_[j,1] < 129.0 and 127.0 < clt.cluster_centers_[j,2] < 129.0:
                        self.centers = np.delete(clt.cluster_centers_, j, 0)
                        break

                #reshape image
                self.cluster[i] = self.cluster[i].reshape((h, w, 3))

                #create masks to determine how many pixels are each color. The higher number is the target color and the lower is the letter color
                c1 = cv.inRange(self.cluster[i], self.centers[0,:]-1, self.centers[0,:]+1)
                c2 = cv.inRange(self.cluster[i], self.centers[1,:]-1, self.centers[1,:]+1)
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
                        self.cluster[i] = None
                        break

                d_centers = dist.euclidean(self.centers[0], self.centers[1])
                if d_centers < 25:#15
                    self.cluster[i] = None
                    #print('Cluster %i failed: Clustered too close' % (i))
                    continue

                #NOTE: now changes letter to white and background to black
                if self.cluster[i] is not None:
                    self.cluster[i][np.where((self.cluster[i].astype(int)==self.centers[0,:].astype(int)).all(axis=2))] = [0,128,128]
                    self.cluster[i][np.where((self.cluster[i].astype(int)==self.centers[1,:].astype(int)).all(axis=2))] = [255,128,128]
                    self.cluster[i] = cv.cvtColor(self.cluster[i], cv.COLOR_LAB2BGR)

                    self.cluster[i] = cv.cvtColor(self.cluster[i], cv.COLOR_BGR2GRAY)
                    cnts = cv.findContours(self.cluster[i].copy(), cv.RETR_CCOMP,
                        cv.CHAIN_APPROX_SIMPLE)
                    contours = cnts[1]
                    hierarchy = cnts[2]

                    #if there is at least one countour,
                    if len(contours) > 0:
                        #find the biggest contour
                        areas = [cv.contourArea(c) for c in contours] # get the area of each contour
                        max_index = np.argmax(areas)
                        c = max(contours, key=cv.contourArea)
                        #remake the mask with only the biggest contour and show
                        self.cluster[i][:,:] = 0

                        cv.drawContours(self.cluster[i], [c], 0, 255, cv.FILLED)
                        child = hierarchy[0][max_index][2]
                        if child != -1:
                            cv.drawContours(self.cluster[i], [contours[child]], 0, 0, cv.FILLED)

                        self.cluster[i] = cv.bitwise_not(self.cluster[i])

                        #Two false positive checks:
                        #   If a large portion of the edge of the "letter" is on the border
                        #   If the "letter" isn't big enough related to the shape
                        letter_edge = cv.Canny(self.cluster[i], 10, 50)
                        letter_edge_count = cv.countNonZero(letter_edge)
                        shape_edge = cv.Canny(self.flood_crop[i], 10, 50)
                        combined_edge = cv.bitwise_and(letter_edge, shape_edge)
                        combined_edge_count = cv.countNonZero(combined_edge)

                        h,w = self.cluster[i].shape
                        letter_area = h*w - cv.countNonZero(self.cluster[i])
                        shape_area = cv.countNonZero(self.flood_crop[i])

                        if combined_edge_count/letter_edge_count > 0.1 or letter_area/shape_area < 0.05:
                            #print("Cluster %i failed: Letter not likely" % (i))
                            self.cluster[i] = None


                    if show and self.cluster[i] is not None:
                        cv.imshow('Quantized %i' % (i), self.cluster[i])

                    #Deterimine colors
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
                        print('Color %i %i: %s' % (i,count,self.colorNames[minDist[1]]))

            else:
                self.cluster.append(None)

    def get_shape(self):
        for i in range(0, len(self.cluster)):
            if self.cluster[i] is not None:
                peri = cv.arcLength(self.c[i], True)
                approx = cv.approxPolyDP(self.c[i], 0.02 * peri, True)
                #print('Shape %i: %i' % (i,len(approx)))

    def classify(self, show=False):

        #print("crops: %i" % len(self.img_crop))
        self.get_mask(show)
        self.get_white_back(show)
        self.color_cluster(show)
        self.get_shape()

    def clear_lists(self):

        self.img_crop.clear()
        self.blur_crop.clear()
        self.canny_crop.clear()
        self.flood_crop.clear()
        self.c.clear()
        self.big_contour.clear()
        self.white_back.clear()
        self.cluster.clear()

#Possible Fail Cases:
#   No countour found in the edge detector of this class
#   No two obviously different colors found (k_means centers are too close together)
#   No possible shape detected
#   No letter detected
#
# detector = AutonomousDetection()
# classify = AutonomousClassification()
# img_crops = []
#
# count = 0
# for i in range(0,1):
#     img_crops.clear()
#     img = cv.imread('../test_target_images/field_targets2.jpg')
#     #img = cv.imread('../snow_target_images/snow%i.jpg' % (i+1))
#     #img = cv.imread('../flight_pics/test%i.jpg' % (i))
#     #img = cv.imread('../comp_target_images/target1.jpg')
#     #img = cv.imread('../comp_no_target_images/field6.jpg')
#     start = time.time()
#     img_crops = detector.detect(img,1)
#     print('Detector Time: %f' % (time.time()-start))
#
#     print("Keyponts length: %i" % (len(img_crops)))
#     # cv.waitKey(0)
#     # cv.destroyAllWindows()
#
#     if len(img_crops):
#         start = time.time()
#         classify.load_new_crops(img_crops)
#         classify.classify(1)
#         print('Classify Time: %f' % (time.time()-start))
#         cv.waitKey(0)
#         cv.destroyAllWindows()
