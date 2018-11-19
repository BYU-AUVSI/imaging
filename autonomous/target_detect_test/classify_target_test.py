import numpy as np
import cv2
import imutils
from PIL import Image
import pytesseract
import os


def test_canny(_img, v, d):
    _edges = cv2.Canny(_img, v, v - d)
    cv2.imshow("Canny", _edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def crop_image(_img, _contour, _crop_ratio=16):
    mask = np.zeros_like(_img)
    cv2.drawContours(mask, _contour, 0, 255, -1)  # Draw filled contour in mask

    if len(mask.shape) == 3:
        (y, x, z) = np.where(mask == 255)
    else:
        (y, x) = np.where(mask == 255)
    yoffset = int(_img.shape[0] / _crop_ratio)
    xoffset = int(_img.shape[1] / _crop_ratio)
    (topy, topx) = (np.min(y) - yoffset, np.min(x) - xoffset)
    (bottomy, bottomx) = (np.max(y) + yoffset, np.max(x) + xoffset)
    return _img[topy:bottomy, topx:bottomx]


# Still need to detect trapezoid, star, cross, semicircle, and quarter circle
def detect_target_shape(c):
    # initialize the shape name and approximate the contour
    shape = "unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    num_vert = len(approx)

    # if the shape is a triangle, it will have 3 vertices
    if num_vert == 3:
        shape = "triangle"

    # if the shape has 4 vertices, it is either a square or
    # a rectangle
    elif num_vert == 4:
        # compute the bounding box of the contour and use the
        # bounding box to compute the aspect ratio
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)

        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"

    # if the shape is a pentagon, it will have 5 vertices
    elif num_vert == 5:
        shape = "pentagon"

    elif num_vert == 6:
        shape = "hexagon"

    elif num_vert == 7:
        shape = "heptagon"

    elif num_vert == 8:
        shape = "octagon"

    # otherwise, we assume the shape is a circle
    else:
        shape = "circle"

    # return the name of the shape
    return shape


def get_char_from_image(_img):
    # filename = "{}.png".format(os.getpid())
    # cv2.imwrite(filename, _img)
    config = ('-l eng --oem 1 --psm 10')
    text = pytesseract.image_to_string(_img, config=config)
    # os.remove(filename)
    return text

def equalize_image(_img):
    b, g, r = cv2.split(_img)
    e_b = cv2.equalizeHist(b)
    e_g = cv2.equalizeHist(g)
    e_r = cv2.equalizeHist(r)
    return cv2.merge((e_b, e_g, e_r))

def rotate_img(_img, _angle):
    scale = 1
    (rows, cols) = _img.shape
    M = cv2.getRotationMatrix2D((cols/2, rows/2), _angle, scale)
    return cv2.warpAffine(_img, M, (cols, rows))


img = cv2.imread("target.jpg", 1)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
res, thresh = cv2.threshold(hsv[:, :, 0], 75, 255, cv2.THRESH_BINARY_INV)

_, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
target_contour = []
for c in contours:
    a = cv2.contourArea(c)
    if 7000 < a < 50000:
        target_contour.append(c)

crop = crop_image(img, target_contour)
crop2 = crop_image(img, target_contour, float("inf"))
shape_str = detect_target_shape(target_contour[0])
print("Shape is a: {}".format(shape_str))

img2 = img.copy()
img3 = img.copy()
index = -1
thickness = 4
color = (255, 0, 255)
cv2.drawContours(img2, target_contour, index, color, thickness)
peri = cv2.arcLength(target_contour[0], True)
approx = cv2.approxPolyDP(target_contour[0], 0.04 * peri, True)
hull = cv2.convexHull(target_contour[0])
cv2.drawContours(img3, [hull], index, color, thickness)


equ = equalize_image(crop2)

hsv_equ = cv2.cvtColor(equ, cv2.COLOR_BGR2HSV)
res2, thresh2 = cv2.threshold(hsv_equ[:, :, 0], 160, 255, cv2.THRESH_BINARY_INV)

thresh_adapt = cv2.adaptiveThreshold(cv2.cvtColor(equ, cv2.COLOR_BGR2GRAY), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 45, 5)
thresh_adapt2 = cv2.adaptiveThreshold(cv2.cvtColor(equ, cv2.COLOR_BGR2GRAY), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 61, 21)

_, contours_char, hierarchy = cv2.findContours(thresh_adapt2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
char_contours = []
for c in contours_char:
    a = cv2.contourArea(c)
    if 500 < a < cv2.contourArea(target_contour[0]):
        char_contours.append(c)

crop_char = crop_image(thresh_adapt2, char_contours, 32)
crop_char_color = crop_image(crop2, char_contours, 32)
# equ_char = equalize_image(crop_char)
# hsv_equ_char = cv2.cvtColor(equ_char, cv2.COLOR_BGR2HSV)
# res2, thresh_char = cv2.threshold(hsv_equ_char[:, :, 0], 70, 255, cv2.THRESH_BINARY_INV)
# thresh_adapt_char = cv2.adaptiveThreshold(cv2.cvtColor(equ_char, cv2.COLOR_BGR2GRAY), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY , 51, 7)


maxV = 250
edges = cv2.Canny(thresh2, maxV, maxV-100)
edges_adapt = cv2.Canny(thresh_adapt, maxV, maxV-100)
edges_adapt2 = cv2.Canny(thresh_adapt2, maxV, maxV-100)
crop_char_rotated = rotate_img(crop_char, 12)
edges_char = cv2.Canny(crop_char, maxV, maxV-100)
edges_char_rotated = cv2.Canny(crop_char_rotated, maxV, maxV-100)


print("Character: {}".format(get_char_from_image(crop_char)))

# cv2.imshow("Contours", img2)
# cv2.imshow("Canny", edges)
# cv2.imshow("Canny Adapt", edges_adapt)
# cv2.imshow("Canny Adapt2", edges_adapt2)
# cv2.imshow("Thresh", thresh)
# cv2.imshow("Crop", crop)
# cv2.imshow("Crop2", crop2)
# cv2.imshow("Hull", img3)
# cv2.imshow("Hist", equ)
# cv2.imshow("Thresh_equ", thresh2)
# cv2.imshow("Adaptive Thresh", thresh_adapt)
# cv2.imshow("Adaptive Thresh2", thresh_adapt2)
cv2.imshow("Edges Char", edges_char)
cv2.imshow("Ad Thresh Char", crop_char)
cv2.imshow("Ad Thresh Char Rotate", crop_char_rotated)
# cv2.imshow("Char Color", crop_char_color)



cv2.waitKey(0)
cv2.destroyAllWindows()
