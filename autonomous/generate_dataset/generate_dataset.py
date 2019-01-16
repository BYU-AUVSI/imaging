from PIL import Image, ImageFont, ImageDraw, ImageFilter
from tqdm import tqdm
import math
import os
from random import randint

# srry fam, atm, there some crappy directory stuff that probably makes this incompatible with windows
# PARAMETERS FOR YOU!!
HEIGHT_ = 100
WIDTH_  = 100
X_CENTER_ = WIDTH_  * 0.5
Y_CENTER_ = HEIGHT_ * 0.5
ROTATE_STEPS_ = 8 # Number of rotation steps to make. should be divisibile by 360
BLUR_MAX_ = 3 # blur upper bound. from 0 -> this exclusive
BLUR_STEP_ = 1 # amount of blur to apply at each level
# that's right, in our world, you define the alphabet
ALPHABET_ = list("ABC") #DEFGHIJKLMNOPQRSTUVWXYZ
BASE_PATH_ = 'generated/' #path to store the images in, relative to this 

def drawSquare(imgDraw, fillColor):
    h80 = int(HEIGHT_ * 0.8)
    w80 = int(WIDTH_ *  0.8)
    imgDraw.rectangle(((WIDTH_ - w80, HEIGHT_ - h80), (w80, h80)), fill=fillColor)

def drawTallRect(imgDraw, fillColor):
    w70 = int(WIDTH_  * 0.7)
    h85 = int(HEIGHT_ * 0.85)
    imgDraw.rectangle(((WIDTH_ - w70, HEIGHT_ - h85), (w70, h85)), fill=fillColor)


def drawQuarterCircle(imgDraw, fillColor):
    #currently doesnt work
    h80 = int(HEIGHT_ * 0.8)
    w80 = int(WIDTH_ *  0.8)
    imgDraw.pieslice([(0, 0), (100, 100)], -90, 0, fill=fillColor)
    
COLOR_NAMES_ = [ "black", "red", "green", "blue", "purple", "orange"] #"white",

# rgb values, list of color lists for each color listed above
COLORS_ = [
    # [(255,255,255), (234,234,234)],
    [(0,0,0), (21,21,21)],
    [(255,0,0), (204,51,51)],
    [(0,255,0), (51,204,51)],
    [(0,0,255), (51,51,204)],
    [(204,0,255), (153,51,204)],
    [(255,165,0), (255,102,0)]
]

SHAPES_ = {
    "square": drawSquare,
    "rectangle-tall": drawTallRect,
    # "quarterCircle": drawQuarterCircle
}

# intermediate helper variable calculation:
fnt = ImageFont.truetype('/Library/Fonts/Arial.ttf', 32)
rotateSection = int(360 / ROTATE_STEPS_) # degree range for each rotation. ie: if ROTATE_STEPS_ == 8, then rotate step 1 will be anything from 0-45 deg, step 2 will be a random angle between 45 and 90, etc, etc
pasteLayerWidth =  int(WIDTH_/1.5)
pasteLayerHeight = int(HEIGHT_/1.5)
ttlColors = len(COLOR_NAMES_) * len(COLORS_[0])
ttlRotateSteps = ROTATE_STEPS_
ttlBlurSteps = math.ceil(BLUR_MAX_ / BLUR_STEP_)
ttlShapeOptions = len(SHAPES_) 
currentPath = os.path.dirname(os.path.realpath(__file__))
pbar = tqdm(total=ttlColors * len(ALPHABET_) * ttlRotateSteps * ttlBlurSteps * ttlShapeOptions)

for letter in ALPHABET_:
    savePath = BASE_PATH_ + letter + '/'
    if not os.path.exists(currentPath + '/' + savePath):
        os.makedirs(currentPath + '/' + savePath)
    for letterColor in range(len(COLOR_NAMES_)):
        for subLetterColor in range(len(COLORS_[letterColor])):
            for blur in range(0, BLUR_MAX_, BLUR_STEP_):
                for angleIter in range(1, ROTATE_STEPS_+1):
                    for shape in SHAPES_:

                        shapeImg = Image.new('RGBA', (WIDTH_, HEIGHT_))
                        shapeDrawn = ImageDraw.Draw(shapeImg)
                        # make the shape a random color that isnt the same as the current letter color
                        randColor = randint(0, len(COLORS_)-1)
                        while randColor == letterColor:
                            randColor = randint(0, len(COLORS_)-1)

                        randColorIndex = randint(0, 1)
                        randRotation = randint(0, 359) # for shape we do a purely random rotation every time

                        SHAPES_[shape](shapeDrawn, COLORS_[randColor][randColorIndex])
                        shapeImg = shapeImg.rotate(randRotation, expand=1)
                        shapeImg = shapeImg.filter(ImageFilter.GaussianBlur(radius=blur))
                        wShape, hShape = shapeImg.size

                        letterImg = Image.new('RGBA', (pasteLayerWidth, pasteLayerHeight))
                        drawn = ImageDraw.Draw(letterImg)
                        w, h = drawn.textsize(letter, font=fnt)
                        drawn.text(((pasteLayerWidth-w)/2,(pasteLayerHeight-h)/2), letter, fill=COLORS_[letterColor][subLetterColor], font=fnt)
                        letterAngle = randint(rotateSection * (angleIter-1), rotateSection * angleIter)
                        letterImg = letterImg.rotate(letterAngle, expand=1)
                        letterImg = letterImg.filter(ImageFilter.GaussianBlur(radius=blur))
                        w, h = letterImg.size
                        
                        baseImg = Image.new('RGB', (WIDTH_, HEIGHT_), color = (255,255,255))
                        baseImg.paste(shapeImg, (int((WIDTH_ - wShape)/2) ,int((HEIGHT_ - hShape)/2)), shapeImg)
                        baseImg.paste(letterImg, (int((WIDTH_ - w)/2) ,int((HEIGHT_-h)/2)), letterImg)
                        baseImg.save('{}{}-{}-{}{}-{}deg-blur{}.jpeg'.format(savePath, letter, shape, COLOR_NAMES_[letterColor], subLetterColor, letterAngle, blur))

        
            pbar.update(ttlBlurSteps * ttlRotateSteps * ttlShapeOptions) # update progress bar
    

pbar.close()