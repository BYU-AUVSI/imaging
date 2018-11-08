'''
Autor: Knowles,D.
Date: 6/26/18
'''
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('example.jpg')   #imread('name',0) = grayscale
# reorder to plot in matlab
b,g,r = cv2.split(img)
img2 = cv2.merge([r,g,b])
#print(img)         # if file path is wrong, doesn't auto error
cv2.imshow('BGR',img)
cv2.imshow('RGB',img2)
key_input = cv2.waitKey(0)
while key_input != 27 or key_input != ord('s'):
    if key_input == 27:
        cv2.destroyAllWindows()
        #cv2.destroyWindow('Image Name')
        break
    elif key_input == ord('s'):
        cv2.imwrite('test_image1.jpg',img)
        cv2.destroyAllWindows()
        break
    key_input = cv2.waitKey(0)
