import cv2 as cv
import os
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time

name = "notarget"
save_dir = 'shapes/' + name + '/'

"""
This generator takes an MOV file of a target and runs a basic detector on them
before exporting individual frames as images. This allows quick and easy dataset
generation from a minute or two of video. 

To run change the 'name' variable above. This name is used to find the mov file
and generate names for the final images. ie: 'square' would load 'square.mov'
(in the same directory as this script) and export all the good dataset frames
to shapes/square/squareX.jpg where x is some integer.

If your video isnt generating frames. Make sure the shape is relatively centered
in the video. We first crop and then run detection. You may also need to adjust
the crop to better accommodate your input video.
If you are still having issues, consider adjusting the canny thresholds (cv2.Canny)
as well as the contourArea conditional (cv.contourArea). countour area checks that
the detected/filled blob is big enough within the frame.
"""
currentPath = os.path.dirname(os.path.realpath(__file__))
if not os.path.exists(currentPath + '/' + save_dir):
    os.makedirs(currentPath + '/' + save_dir)
#Import the video
cap = cv.VideoCapture(name + '.mov')

#Uncomment these to save a video
#fourcc = cv.VideoWriter_fourcc(*'XVID')
#out = cv.VideoWriter('output.avi',fourcc, 30.0, (400,200))

#Basic counters
write_count = 0
count = 0

#Loop throuogh the total number of video frames
while cap.isOpened(): # and count < 300 <-- if you want to limit the amount of runtime
    #Read a new frame
    ret, frame = cap.read()
    
    if not ret:
        print("no frame!!")
        break

    #Resize, crop, and resize again to obtain 200x200 image with target in center
    # probably need to adjust this a bit to get the  target properly in frame
    frame_crop = imutils.resize(frame, width = 750)
    frame_crop = frame_crop[20:420, 200:600]
    frame_crop = imutils.resize(frame_crop, width = 200)

    #Blur and posterize. Can change parameters for random blur if desired
    blur = cv.GaussianBlur(frame_crop, (5, 5), 0)
    blur = cv.pyrMeanShiftFiltering(blur, 10, 10, 3)

    #Extract edges
    canny = cv.Canny(blur, 10, 100)
    canny = cv.dilate(canny, None, iterations=1)

    #Create filled mask using edges
    h, w = canny.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    edges = canny.copy()
    cv.floodFill(edges, mask, (0,0), 255)
    edges = cv.bitwise_not(edges)
    flood = (canny | edges)

    #Find the contours in the filled mask
    cnts = cv.findContours(flood.copy(), cv.RETR_EXTERNAL,
        cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[1]

    #If there is at least one countour,
    if len(cnts) > 0:
        #find the biggest contour
        c = max(cnts, key=cv.contourArea)
        #Use print statement to determine size to filter out bad data
        #print(cv.contourArea(c))
        if cv.contourArea(c) > 4000:    #Replace size
            #remake the mask with only the biggest contour
            flood[:,:] = 0
            cv.drawContours(flood, [c], 0, 255, cv.FILLED)
            #erode the mask to eliminate any leftover background
            flood = cv.erode(flood, None, iterations=2)

            # sum along rows and columns, really we're only interested in the 
            # first and last row and column (ie: a box around the image of the last pixels)
            rowSum = np.sum(flood, axis=1)
            colSum = np.sum(flood, axis=0)
            maxDim = len(rowSum) - 1 # should be square

            # dont write if the shape its against the edge of the image
            #   shapes absorbed by the edge wont happen irl, and it screws with 
            #   the target shape, such that we shouldnt be able to id it anyways
            if rowSum[0] < 1 and rowSum[maxDim] < 1 and colSum[0] < 1 and colSum[maxDim] < 1:  
                #Write the mask to a file
                cv.imwrite(save_dir + name + '%i.jpg' % (write_count), flood)
                write_count += 1

    #Show crop and mask
    cv.imshow('frame',frame_crop)
    cv.imshow('mask',flood)
    #Uncomment to write video
    #out.write(np.hstack([frame_crop, cv.cvtColor(flood,cv.COLOR_GRAY2BGR)]))

    #Press q to kill process
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    count += 1

#Clean up
cap.release()
#out.release()
cv.destroyAllWindows()