import cv2 as cv
import numpy as np
import math


temp = cv.imread('letter_temp/i.jpg', cv.IMREAD_GRAYSCALE)
#img = cv.imread('../training_data/letters/A/A47.jpg', cv.IMREAD_GRAYSCALE)
img = cv.imread('letters/letter15.jpg', cv.IMREAD_GRAYSCALE)



minHessian = 400#400
detector = cv.xfeatures2d_SURF.create(hessianThreshold=minHessian)

#detector = cv.xfeatures2d_SIFT.create()
keypoints_obj, descriptors_obj = detector.detectAndCompute(temp, None)
keypoints_scene, descriptors_scene = detector.detectAndCompute(img, None)

# matcher = cv.DescriptorMatcher_create(cv.DescriptorMatcher_FLANNBASED)
# knn_matches = matcher.knnMatch(descriptors_obj, descriptors_scene, 2)
#
# ratio_thresh = 1.0
# good_matches = []
# for m,n in knn_matches:
#     if m.distance < ratio_thresh * n.distance:
#         good_matches.append(m)

matcher = cv.BFMatcher()
matches1 = matcher.match(descriptors_obj,descriptors_scene)
matches2 = matcher.match(descriptors_scene,descriptors_obj)

good_matches = []
result1 = []

# get putative matches
for match in matches2:
    result1.append([keypoints_scene[match.queryIdx].pt, keypoints_obj[match.trainIdx].pt])
for match2 in matches1:
    candidate = [keypoints_scene[match2.trainIdx].pt, keypoints_obj[match2.queryIdx].pt]
    if candidate in result1:
        good_matches.append(match2)

good_matches = sorted(good_matches, key = lambda x:x.distance)




img_matches = np.empty((max(temp.shape[0], img.shape[0]), temp.shape[1]+img.shape[1], 3), dtype=np.uint8)
cv.drawMatches(temp, keypoints_obj, img, keypoints_scene, good_matches, img_matches, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

obj = np.empty((len(good_matches),2), dtype=np.float32)
scene = np.empty((len(good_matches),2), dtype=np.float32)
for i in range(len(good_matches)):
    #-- Get the keypoints from the good matches
    obj[i,0] = keypoints_obj[good_matches[i].queryIdx].pt[0]
    obj[i,1] = keypoints_obj[good_matches[i].queryIdx].pt[1]
    scene[i,0] = keypoints_scene[good_matches[i].trainIdx].pt[0]
    scene[i,1] = keypoints_scene[good_matches[i].trainIdx].pt[1]
#H, _ =  cv.findHomography(scene, obj, cv.LMEDS)
H = cv.estimateRigidTransform(scene,obj,True)

print(H)

theta = math.atan2(H[1,0], H[1,1]) * 180 / math.pi
print(theta)

M = cv.getRotationMatrix2D((img.shape[1]/2,img.shape[0]/2),-theta,1)
img_out = cv.warpAffine(img,M,(img.shape[1],img.shape[0]))

#img_out = cv.warpPerspective(img, H, (img.shape[1],img.shape[0]))

cv.imshow('Warped', img_out)
cv.imshow('Good Matches & Object detection', img_matches)
cv.waitKey()
