
"""
Most of this is a port of @BrandonMcbride 's generate_dataset.jsx photoshop
    script to work with PIL. He did all the hard math here

This was the original dataset generator, which we ended up not using. Images are
generated entirely synthetically, meaning you can get a whole lot really fast.
The main issue is that the shapes are too perfect and are only seen from a top-down
perspective. Training with this dataset lead to a net that performed great on
this type of data, but utterly failed on real-world results. However,
some of the code/principles from this are still useful, which is why it's still
included in the repo. You may find a need to use some or part of this code in the future
"""

from PIL import Image, ImageFont, ImageDraw, ImageFilter
from tqdm import tqdm
from skimage.util import random_noise
import numpy as np
import math
import os
from random import randint

DIM_ = 200
HEIGHT_ = DIM_ # things will probably break if it isnt a square img??
WIDTH_  = DIM_
FONT_PT_ = 52
X_CENTER_ = WIDTH_  * 0.5
Y_CENTER_ = HEIGHT_ * 0.5
ROTATE_STEPS_ = 8 # Number of rotation steps to make. should be divisibile by 360
# that's right, in our world, you define the alphabet
ALPHABET_ = list("A") #BCDEFGHIJKLMNOPQRSTUVWXYZ
BASE_PATH_ = 'generated/' #path to store the images in, relative to this 

def getPolygonCoordinates(sides, radius, theta):
    """
    sides: number of sides of polygon
    r: radius of polygon
    theta: degrees by which the polygon is rotated
    if theta is 0, the first point is directly right of the center
    """
    coordinates = []
    theta = theta * math.pi / 180
    for i in range(0, sides):
        x = radius * math.cos(2 * math.pi * i / sides - theta) + X_CENTER_
        y = radius * math.sin(2 * math.pi * i / sides - theta) + Y_CENTER_
        coordinates.append((x, y))

    return coordinates

def drawSquare(imgDraw, fillColor):
    h80 = int(HEIGHT_ * 0.7)
    w80 = int(WIDTH_ *  0.7)
    imgDraw.rectangle(((WIDTH_ - w80, HEIGHT_ - h80), (w80, h80)), fill=fillColor)

def drawTallRect(imgDraw, fillColor):
    w70 = int(WIDTH_  * 0.7)
    h85 = int(HEIGHT_ * 0.85)
    imgDraw.rectangle(((WIDTH_ - w70, HEIGHT_ - h85), (w70, h85)), fill=fillColor)

def drawSemiCircle(imgDraw, fillColor):
    coordinates = [] # list of tuples for coordinates to draw 
    radius = DIM_*0.3
    r2 = (4*(radius)) / (3 * math.pi)

    iter = 1000
    for i in range(0, iter): # give plenty of points to define circle
        x = int(radius * math.cos(math.pi * i / iter) + X_CENTER_)
        y = int(radius * math.sin(math.pi * i / iter) + Y_CENTER_ - r2)
        coordinates.append((x,y))
    
    imgDraw.polygon(coordinates, fill=fillColor)

def drawQuarterCircle(imgDraw, fillColor):
    coordinates = [] # list of tuples for coordinates to draw 
    radius = DIM_*0.45
    r2 = (4*radius) / (3 * math.pi)
    coordinates.append((X_CENTER_-r2, Y_CENTER_-r2))

    iter = 1000
    for i in range(0, iter): # give plenty of points to define circle
        x = int(radius * math.cos(math.pi / 2.0 * i / iter) + X_CENTER_ - r2)
        y = int(radius * math.sin(math.pi / 2.0 * i / iter) + Y_CENTER_ - r2)
        coordinates.append((x,y))
    imgDraw.polygon(coordinates, fill=fillColor)

def drawCircle(imgDraw, fillColor):
    r = DIM_*0.25
    imgDraw.ellipse((X_CENTER_-r, Y_CENTER_-r, X_CENTER_+r, Y_CENTER_+r), fill=fillColor)

def drawCross(imgDraw, fillColor):
    width = DIM_ * 0.8
    x  = X_CENTER_
    y  = Y_CENTER_
    w2 = width * 0.5
    w8 = width * 0.125
    
    coordinates = [
        (x - w8, y - w2), (x + w8, y - w2), (x + w8, y - w8), (x + w2, y - w8),
        (x + w2, y + w8), (x + w8, y + w8), (x + w8, y + w2), (x - w8, y + w2),
        (x - w8, y + w8), (x - w2, y + w8), (x - w2, y - w8), (x - w8, y - w8)
    ]

    imgDraw.polygon(coordinates, fill=fillColor)

def drawStar(imgDraw, fillColor):
    w = DIM_ * 0.8 # width
    x = X_CENTER_
    y = Y_CENTER_
    w2 = w * 0.5

    coordinates =  [((x), (y - w2)), ((x + 0.235 * w2), (y - 0.235 * w2)), ((x + w2), (y - 0.235 * w2)), ((x + 0.38 * w2), (y + 0.235 * w2)),
        ((x + 0.62 * w2), (y + w2)), ((x), (y + 0.52 * w2)), ((x - 0.6167 * w2), (y + w2)), ((x - 0.38 * w2), (y + 0.235 * w2)),
        ((x - w2), (y - 0.235 * w2)), ((x - 0.235 * w2), (y - 0.235 * w2))]

    imgDraw.polygon(coordinates, fill=fillColor)

def drawTriangle(imgDraw, fillColor):
    b2 = (DIM_*0.7) * 0.5
    h2 = (DIM_*0.75) * 0.5
    x = X_CENTER_
    y = Y_CENTER_
    coordinates =  [(x, y - h2), (x + b2, y + (DIM_*0.25)), (x - b2, y + (DIM_*0.25))]
    imgDraw.polygon(coordinates, fill=fillColor)

def drawTrapezoid(imgDraw, fillColor):
    w12 = DIM_ * 0.2
    w22 = DIM_ * 0.35
    h2 = DIM_ * 0.2
    x = X_CENTER_
    y = Y_CENTER_
    coordinates =  [(x - w12, y - h2), (x + w12, y - h2), (x + w22, y + h2), (x - w22, y + h2)]
    imgDraw.polygon(coordinates, fill=fillColor)

def drawHexagon(imgDraw, fillColor):
    imgDraw.polygon(getPolygonCoordinates(6, DIM_*0.25, 0), fill=fillColor)

def drawPentagon(imgDraw, fillColor):
    imgDraw.polygon(getPolygonCoordinates(5, DIM_*0.25, 90), fill=fillColor)

def drawHeptagon(imgDraw, fillColor):
    imgDraw.polygon(getPolygonCoordinates(7, DIM_*0.25, 90), fill=fillColor)

def drawOctagon(imgDraw, fillColor):
    imgDraw.polygon(getPolygonCoordinates(8, DIM_*0.25, 0), fill=fillColor)

COLOR_NAMES_ = ["white", "black", "gray", "red", "green", "blue", "yellow", "brown", "purple", "orange"]

# rgb values, list of color lists for each color listed above
COLORS_ = [
    [(247, 247, 247), (240, 240, 240), (234, 234, 234), (217, 217, 217)], # white
    [(0, 0, 0), (5, 5, 5), (10, 10, 10), (16, 16, 16), (21, 21, 21), 
        (26, 26, 26), (32, 32, 32), (37, 37, 37), (42, 42, 42)], # black
    [(80, 80, 80), (85, 85, 85), (90, 90, 90), (96, 96, 96), 
        (101, 101, 101), (106, 106, 106), (112, 112, 112), 
        (117, 117, 117), (122, 122, 122), (128, 128, 128), 
        (133, 133, 133), (138, 138, 138), (144, 144, 144), 
        (149, 149, 149), (154, 154, 154), (160, 160, 160), 
        (165, 165, 165)], # gray
    [(255, 0, 0), (255, 42, 0), (255, 0, 51), (255, 51, 51), 
        (255, 0, 102), (255, 51, 102), (255, 102, 102), (255, 0, 153), 
        (255, 51, 153), (255, 102, 153), (255, 153, 153), (255, 204, 204), 
        (204, 0, 0), (204, 16, 0), (204, 0, 51), (204, 51, 51), 
        (204, 0, 102), (204, 51, 102), (153, 0, 0), (153, 0, 51), 
        (153, 51, 51), (102, 0, 0)], # red
    [(0, 255, 0), (51, 255, 0), (0, 255, 51), (51, 255, 51), (102, 255, 0), 
        (0, 255, 102), (102, 255, 102), (0, 255, 153), (153, 255, 0), 
        (153, 255, 153), (0, 255, 204), (0, 204, 0), (51, 204, 0), (0, 204, 51), 
        (51, 204, 51), (102, 204, 0), (0, 204, 102), (102, 204, 102), 
        (0, 204, 153), (153, 255, 0), (0, 153, 0), (51, 153, 0), (0, 153, 51), 
        (51, 153, 51), (102, 153, 0), (0, 153, 102), (102, 153, 102), 
        (0, 102, 0), (51, 102, 0), (0, 102, 51), (51, 102, 51)], # green
    [(0, 0, 255), (51, 51, 255), (0, 51, 255), (51, 0, 255), (102, 102, 255), 
        (0, 102, 255), (0, 128, 255), (0, 153, 255), (51, 153, 255), 
        (102, 153, 255), (0, 204, 255), (51, 204, 255), (102, 178, 255), 
        (102, 204, 255), (0, 0, 204), (51, 0, 204), (0, 51, 204), 
        (51, 51, 204), (0, 102, 204), (51, 102, 204), (0, 153, 204), 
        (51, 153, 204), (102, 153, 204), (0, 0, 153), (51, 0, 153), 
        (0, 51, 153), (51, 51, 153), (0, 102, 153), (51, 102, 153), 
        (0, 153, 153), (51, 153, 153), (0, 0, 102), (0, 51, 102)], # blue
    [(255, 255, 0), (255, 255, 51), (255, 255, 102), (255, 255, 153), 
        (255, 255, 204), (255, 204, 0), (255, 204, 51), (255, 204, 102), 
        (204, 204, 0), (204, 204, 51), (204, 204, 102), (204, 204, 153), 
        (204, 153, 0), (204, 153, 51), (153, 153, 0), (153, 153, 51), 
        (102, 102, 0), (102, 102, 51)], # yellow
    [(139, 69, 19), (160, 82, 45), (205, 133, 63), (210, 105, 30), 
        (153, 102, 0), (204, 102, 0)], # brown
    [(255, 0, 255), (255, 51, 255), (255, 102, 255), (255, 153, 255), 
        (204, 0, 255), (204, 51, 255), (204, 102, 255), (204, 153, 255), 
        (153, 0, 255), (153, 51, 255), (153, 102, 255), (153, 153, 255), 
        (153, 0, 204), (153, 51, 204), (153, 102, 204), (102, 0, 204), 
        (102, 51, 204), (102, 102, 204)], #purple
    [(255, 187, 0), (255, 187, 51), (255, 187, 102), (255, 153, 0), 
        (255, 153, 51), (255, 153, 102), (255, 102, 0), (255, 102, 51), 
        (255, 69, 0), (255, 69, 51), (204, 102, 51)] # orange
]

SHAPES_ = {
    "square": drawSquare,
    "rectangle": drawTallRect,
    "semicircle": drawSemiCircle,
    "quarterCircle": drawQuarterCircle,
    "circle": drawCircle,
    "cross": drawCross,
    "star": drawStar,
    "triangle": drawTriangle,
    "trapezoid": drawTrapezoid,
    "hexagon": drawHexagon,
    "heptagon": drawHeptagon,
    "octagon": drawOctagon,
    "pentagon": drawPentagon,
}

# intermediate helper variable calculation:
fnt = ImageFont.truetype('assets/fonts/Arial Bold.ttf', FONT_PT_)
rotateSection = int(360 / ROTATE_STEPS_) # degree range for each rotation. ie: if ROTATE_STEPS_ == 8, then rotate step 1 will be anything from 0-45 deg, step 2 will be a random angle between 45 and 90, etc, etc
pasteLayerWidth =  int(WIDTH_/1.5)
pasteLayerHeight = int(HEIGHT_/1.5)
COLOR_CHANGES_PER_COLOR = 8 # number of colors todo for each base color
ttlColors = len(COLOR_NAMES_) * COLOR_CHANGES_PER_COLOR
ttlRotateSteps = ROTATE_STEPS_
ttlShapeOptions = len(SHAPES_) 
currentPath = os.path.dirname(os.path.realpath(__file__))
pbar = tqdm(total=ttlColors * len(ALPHABET_) * ttlRotateSteps * ttlShapeOptions)

for letter in ALPHABET_:
    for letterColor in range(len(COLOR_NAMES_)): # for each base color
        for subLetterColor in range(COLOR_CHANGES_PER_COLOR): # number of times todo a random color in base color array (ie: do 4 random 'orange' letters)
            for angleIter in range(1, ROTATE_STEPS_+1):
                for shape in SHAPES_:

                    # setup folder to save in if necessary
                    savePath = BASE_PATH_ + letter + '-' + shape + '/'
                    if not os.path.exists(currentPath + '/' + savePath):
                        os.makedirs(currentPath + '/' + savePath)

                    shapeImg = Image.new('RGBA', (WIDTH_, HEIGHT_))
                    shapeDrawn = ImageDraw.Draw(shapeImg)
                    # make the shape a random color that isnt the same as the current letter color
                    randColor = randint(0, len(COLORS_)-1)

                    randLetterColorIndex = randint(0, len(COLORS_[letterColor])-1)

                    randShapeColorIndex = randint(0, len(COLORS_[randColor])-1)
                    while randColor == letterColor and randShapeColorIndex == randLetterColorIndex:
                        # just make sure we aren't using the EXACT same color for both shape and letter
                        randShapeColorIndex = randint(0, len(COLORS_[randColor])-1)

                    randRotation = randint(0, 359) # for shape we do a purely random rotation every time

                    # draw the actual shape
                    SHAPES_[shape](shapeDrawn, COLORS_[randColor][randShapeColorIndex])
                    
                    # apply random noise
                    # shape_im_arr = np.frombuffer(shapeImg.tobytes(), dtype=np.uint8)
                    # shape_im_arr = shape_im_arr.reshape((shapeImg.size[1], shapeImg.size[0], 4))    
                    # shape_im_arr = random_noise(shape_im_arr, mode='localvar', seed=None, clip=True)
                    # shapeImg = Image.fromarray(255*shape_im_arr.astype('uint8'), 'RGBA') # convert back to PIL image
                    # shapeImg = shapeImg.filter(ImageFilter.GaussianBlur(radius=2)) # blur the noise so it looks normal

                    # shapeImg = shapeImg.rotate(randRotation, expand=1)
                    wShape, hShape = shapeImg.size

                    # draw letter
                    letterImg = Image.new('RGBA', (pasteLayerWidth, pasteLayerHeight))
                    drawn = ImageDraw.Draw(letterImg)
                    w, h = drawn.textsize(letter, font=fnt)
                    drawn.text(((pasteLayerWidth-w)/2,(pasteLayerHeight-h)/2), letter, fill=COLORS_[letterColor][randLetterColorIndex], font=fnt)
                    # rotate letter, get a random angle for the letter in the bounds of our current rotation angle
                    letterAngle = randint(rotateSection * (angleIter-1), rotateSection * angleIter)
                    letterImg = letterImg.rotate(letterAngle, expand=1)
                    w, h = letterImg.size
                    
                    baseImg = Image.new('RGB', (WIDTH_, HEIGHT_), color = (255,255,255))
                    baseImg.paste(shapeImg, (int((WIDTH_ - wShape)/2) ,int((HEIGHT_ - hShape)/2)), shapeImg)
                    baseImg.paste(letterImg, (int((WIDTH_ - w)/2) ,int((HEIGHT_-h)/2)), letterImg)
                    baseImg.save('{}{}-{}-{}{}_on_{}{}-{}deg.jpeg'.format(savePath, letter, shape, COLOR_NAMES_[letterColor], randLetterColorIndex, COLOR_NAMES_[randColor], randShapeColorIndex, letterAngle))

        
            pbar.update(ttlRotateSteps * ttlShapeOptions) # update progress bar
    

pbar.close()