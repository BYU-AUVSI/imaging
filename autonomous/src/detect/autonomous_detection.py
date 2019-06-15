import cv2 as cv
import numpy as np
import imutils

#Params to tune:
#   Size of blobs: params.minArea/params.maxArea    35-36
#   Size of image: resized_image width              74
#   Canny threshold                                 83
#   Mean shift iterations                           111
#   Min distance between keypoints                  144

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
        params.minArea = 20#200
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
        img_y, img_x, _ = img.shape
        if img_y > img_x:
            img = imutils.rotate_bound(img,90)

        self.original_image = img
        self.resized_image = imutils.resize(img, width=600) #bassler images
        #self.resized_image = imutils.resize(img, width=1200) #sony images

        self.y_orig, self.x_orig, _ = img.shape
        self.y_resize, self.x_resize, _ = self.resized_image.shape

        self.img_crop.clear()

        self.preprocess()
        self.detect_ROIs(edge_limit=500)

        if len(self.keypoints) > 5:
            self.keypoints_image = self.resized_image
            return []

        self.crop_ROIs()

        # refined_crops = self.img_crop.copy()
        # self.img_crop.clear()
        # for i in range(len(refined_crops)):
        #     if refined_crops[i] is not None:
        #         self.img_crop.append(refined_crops[i])


        self.keypoints_image = cv.drawKeypoints(self.resized_image, self.keypoints,
            np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if show:
            # cv.imshow('Resized', self.resized_image)
            # cv.imshow('Preprocess', self.preprocessed)
            # cv.imshow('Canny', self.canny)
            # cv.imshow('Flood', self.flood)
            cv.imshow('Detection', self.keypoints_image)
            key = cv.waitKey(1) & 0xFF
            # for i in range(len(self.img_crop)):
            #     cv.imshow('Cropped Image %i' % (i), self.img_crop[i].crop)
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
                        if dist < 50:#100
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

                ref_crop,bounds = self.refine_crop(crop)

                if ref_crop is not None:

                    bottomY = topY + bounds[1]
                    rightX = leftX + bounds[3]
                    topY += bounds[0]
                    leftX += bounds[2]

                    detected = DetectedCrop(ref_crop, (topY, leftX), (bottomY, rightX))
                    self.img_crop.append(detected)


    def refine_crop(self, crop):

        blur_crop = cv.GaussianBlur(crop, (5, 5), 0)     #**IS THIS NECESSARY?**
        blur_crop = cv.pyrMeanShiftFiltering(blur_crop, 30, 30, 3)
        #cv.imshow('Blur %i' % (i), self.blur_crop[i])

        #detect edges and show
        canny_crop = cv.Canny(blur_crop,10,200)
        #dilating the edges often closes edges that were originally not connected
        canny_crop = cv.dilate(canny_crop, None, iterations=1)
        #self.canny_crop[i] = cv.erode(self.canny_crop[i], None, iterations=1)
        #cv.imshow('Canny %i' % (i), self.canny_crop[i])

        #fill the enclosed edges to create a mask and show
        h, w = canny_crop.shape[:2]
        canny_crop[0,:] = 0
        canny_crop[h-1,:] = 0
        canny_crop[:,0] = 0
        canny_crop[:,w-1] = 0
        mask = np.zeros((h+2, w+2), np.uint8)
        edges = canny_crop.copy()
        cv.floodFill(edges, mask, (0,0), 255)
        edges = cv.bitwise_not(edges)
        flood_crop = canny_crop | edges
        flood_crop = cv.erode(flood_crop, None, iterations=2)

        #find the contours in the filled mask
        cnts = cv.findContours(flood_crop.copy(), cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[1]

        #if there is at least one countour,
        if len(cnts) > 0:
            #find the biggest contour
            c = max(cnts, key=cv.contourArea)
            #remake the mask with only the biggest contour and show
            flood_crop[:,:] = 0

            cv.drawContours(flood_crop, [c], 0, 255, cv.FILLED)
            #erode the mask to eliminate any leftover background
            #self.flood_crop[i] = cv.erode(self.flood_crop[i], None, iterations=2)

            M = cv.moments(c)
            if M["m00"] == 0:
                print('Divide by zero error in detector')
                return None, None

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            _,_,w,h = cv.boundingRect(c)

            buf = 20
            crp = max([w,h])//2+buf

            top = max(1,cY-crp)
            bottom = cY+crp
            left = max(1,cX-crp)
            right = cX+crp

            return crop[top:bottom, left:right], [top,bottom,left,right]

        else:
            return None, None
