import cv2 as cv
import os
import numpy as np
from sklearn.cluster import MiniBatchKMeans
from matplotlib import pyplot as plt
from collections import OrderedDict
from scipy.spatial import distance as dist
import imutils
import time
import argparse

# directory for where the output goes
BASE_SAVE_DIR = "letters"

"""
This generator takes an MOV file of a letter and runs a basic detector on each frame
before exporting them individually as images. This allows quick and easy dataset
generation from a minute or two of video. 

To run change the 'name' variable above. This name is used to find the mov file
and generate names for the final images. ie: 'A' would load 'A.mov'
(in the same directory as this script) and export all the good dataset frames
to letters/A/Ax.jpg where x is some integer.

If your video isnt generating frames. Make sure the letter is relatively centered
in the video. We first crop and then run detection. You may also need to adjust
the crop to better accommodate your input video.
If you are still having issues, consider adjusting the canny thresholds (cv2.Canny)
as well as the contourArea conditional (cv.contourArea). countour area checks that
the detected/filled blob is big enough within the frame.
"""
def generateImages(videoFile, className, cutoff):
    """
    @param videoFile: relative path from this script to the video file to run. 
        Video files should be named after the class they represent (ie: E.MOV for the E class)
    @param className: the name of the class we're exporting from the video (ie: E for a video of the letter E)
    @param cutoff: stop saving images and close the video after this many images have been saved
    """
    print("START:: {}".format(name))
    currentPath = os.path.dirname(os.path.realpath(__file__))
    save_dir = os.path.join(currentPath, BASE_SAVE_DIR, className)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

        
    #Import the video
    cap = cv.VideoCapture(videoFile)

    #Uncomment these to save a video
    #fourcc = cv.VideoWriter_fourcc(*'XVID')
    #out = cv.VideoWriter('output.avi',fourcc, 30.0, (400,200))

    #Basic counters
    write_count = 0
    count = 0

    #Loop throuogh the total number of video frames
    while cap.isOpened(): # and count < 300: # <-- if you want to limit the amount of runtime
        #Read a new frame
        ret, frame = cap.read()

        if not ret:
            print("done!")
            break

        #Resize, crop, and resize again to obtain 200x200 image with target in center
        # probably need to adjust this a bit to get the  target properly in frame
        #frame_crop = imutils.resize(frame, width = 750)
        # print(frame_crop.shape)
        #frame_crop = frame_crop[20:420, 200:600]

        frame_crop = imutils.resize(frame, height = 600)

        frame_crop = frame_crop[0:600, 200:800]
        # print(frame_crop.shape)
        # break
        frame_crop = imutils.resize(frame_crop, width = 200)

        # cv.waitKey(0)
        # cv.destroyAllWindows()
        #Blur and posterize. Can change parameters for random blur if desired
        blur = cv.GaussianBlur(frame_crop, (5, 5), 0)
        blur = cv.pyrMeanShiftFiltering(blur, 10, 10, 3)

        #Extract edges
        canny = cv.Canny(blur, 10, 600)
        canny = cv.dilate(canny, None, iterations=1)

        #Create filled mask using edges
        h, w = canny.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        edges = canny.copy()
        cv.floodFill(edges, mask, (0,0), 255)
        edges = cv.bitwise_not(edges)
        flood = (canny | edges)

        #Find the contours in the filled mask
        cnts = cv.findContours(flood.copy(), cv.RETR_CCOMP,
            cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[1]
        #hierarchy = cnts[2]

        #If there is at least one countour,
        if len(cnts) > 0:
            #find the biggest contour
            c = max(cnts, key=cv.contourArea)
            #Use print statement to determine size to filter out bad data
            # print(cv.contourArea(c))
            if cv.contourArea(c) > 1300:    #Replace size
                #remake the mask with only the biggest contour
                flood[:,:] = 0
                cv.drawContours(flood, [c], 0, 255, cv.FILLED)
                #erode the mask to eliminate any leftover background
                flood = cv.erode(flood, None, iterations=2)

                # sum along rows and columns, really we're only interested in the
                # first and last row and column (ie: a box around the image of the last pixels)
                gray_crop = cv.cvtColor(frame_crop, cv.COLOR_BGR2GRAY)
                letter_only = cv.bitwise_and(flood, gray_crop)
                letter_only[np.where((letter_only.astype(int)==0))] = 255
                _,letter_only = cv.threshold(letter_only, 100, 255, cv.THRESH_BINARY)
                kernel = np.ones((5,5),np.uint8)
                letter_only = cv.bitwise_not(cv.morphologyEx(cv.bitwise_not(letter_only), cv.MORPH_CLOSE, kernel))

                imageSum = np.sum(letter_only)
                allWhiteImage = 255 * letter_only.shape[0] * letter_only.shape[1]
                if imageSum > (allWhiteImage * 0.985):
                    # if the letter only image is > 98.5% white, we dont have a letter,
                    # skip to the next..
                    continue

                rowSum = np.sum(letter_only, axis=1)
                colSum = np.sum(letter_only, axis=0) 
                whiteBorder = 255 * len(rowSum) # and entirely white border is what we're expecting (assuming square img)

                # dont write if the shape its against the edge of the image
                #   shapes absorbed by the edge wont happen irl, and it screws with
                #   the target shape, such that we shouldnt be able to id it anyways
                if rowSum[0] == whiteBorder and rowSum[-1] == whiteBorder and colSum[0] == whiteBorder and colSum[-1] == whiteBorder:
                    #Write the mask to a file
                    saveName = os.path.join(save_dir, "{}{}.jpg".format(className, write_count))
                    cv.imwrite(saveName, letter_only)
                    write_count += 1

                    if cutoff is not None and write_count >= cutoff:
                        cap.release()
                        #out.release()
                        cv.destroyAllWindows()
                        return

                    #Show crop and mask
                    cv.imshow('frame',frame_crop)
                    cv.imshow('letter',letter_only)
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


# parse required input argument
parser = argparse.ArgumentParser(description="Letter Video Dataset Generator")
parser.add_argument("video", help="Relative path to a video or folder of video to use for dataset generation (videos should be named after the class they represent ie: E.MOV -> letters/E/imgs.jpg)")
parser.add_argument('-C', '--cutoff', metavar='cutoff', help="Stop exporting images from a video after [cutoff] images has been reached for that class. Must be an int.")
args = parser.parse_args()

cutoff = None
if args.cutoff is not None:
    temp = int(args.cutoff)
    if temp > 0:
        cutoff = temp
    else:
        print("WARN:: Invalid value for cutoff specified (did you use an integer?), proceeding with not cutoff value")

print("Setting up...")
if os.path.isdir(args.video):
    # if its a folder of videos go through each one
    for vidFile in sorted(os.listdir(args.video)):
        if vidFile.endswith(".MOV") or vidFile.endswith(".mov"):
            name = vidFile.split(".")[0]
            generateImages(os.path.join(args.video, vidFile), name, cutoff)
    exit(1)
else:
    # we're working with a single video file
    if args.video.endswith(".MOV") or args.video.endswith(".mov"):
        name = args.video.split(".")[-2].split("/")[-1] # extract the letter name from the video
        generateImages(args.video, name, cutoff)
    else:
        print("Unrecognized video file format!")
