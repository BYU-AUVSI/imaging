import cv2 as cv
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time

#Import the video
cap = cv.VideoCapture('test.mov')

#Uncomment these to save a video
#fourcc = cv.VideoWriter_fourcc(*'XVID')
#out = cv.VideoWriter('output.avi',fourcc, 30.0, (400,200))

#Basic counters
write_count = 0
count = 0

#Loop throuogh the total number of video frames
while count < 300 and cap.isOpened():
    #Read a new frame
    ret, frame = cap.read()
    #Resize, crop, and resize again to obtain 200x200 image with target in center
    frame = imutils.resize(frame, width = 600)
    frame_crop = frame[400:950, 50:600]
    frame_crop = imutils.resize(frame_crop, width = 200)

    #Blur and posterize. Can change parameters for random blur if desired
    blur = cv.GaussianBlur(frame_crop, (5, 5), 0)
    blur = cv.pyrMeanShiftFiltering(blur, 10, 10, 3)

    #Extract edges
    canny = cv.Canny(blur, 10, 300)
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
        if cv.contourArea(c) > 6000:    #Replace size
            #remake the mask with only the biggest contour
            flood[:,:] = 0
            cv.drawContours(flood, [c], 0, 255, cv.FILLED)
            #erode the mask to eliminate any leftover background
            flood = cv.erode(flood, None, iterations=2)
            #Write the mask to a file
            cv.imwrite('data/trap%i.jpg' % (write_count), flood)
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
