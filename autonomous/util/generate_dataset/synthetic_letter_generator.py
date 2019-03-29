from PIL import Image, ImageFont, ImageDraw, ImageFilter
from tqdm import tqdm
import os
from random import randint

"""
Generates a dataset of letters using various fonts, font sizes, rotations and blurs.

Desirable fonts are those which are clearly uppercase, sans-serif, and have some
degree of distortion or erosion in the letter. This best mimics what is provided 
by the actual detector.

This is untested, but should be platform independent
"""

DIM_ = 200
HEIGHT_ = DIM_ # things will probably break if it isnt a square img??
WIDTH_  = DIM_
NUM_FONT_PTS_ = 3 # number of random font sizes to use for each font
FONT_PTS_ = list(range(80, 121)) # font sizes to use in generation
ROTATE_STEPS_ = 10 # Number of rotation steps to make. should be divisible by 360
BLURS_ = list(range(0, 4)) # which blurs todo. inclusive to exclusive (ie 0-4 will have 0,1,2,3)
# that's right, in our world, you define the alphabet
ALPHABET_ = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ") #BCDEFGHIJKLMNOPQRSTUVWXYZ
BASE_PATH_ = 'generated' #path to store the images in, relative to this 

fontBasePath = os.path.join('assets', 'fonts')
FONTS_ = os.listdir(fontBasePath)
FONTS_.remove('.DS_Store') # remove troublesome non-font files if present
# helper calculations:
rotateSection = int(360 / ROTATE_STEPS_) # degree range for each rotation. ie: if ROTATE_STEPS_ == 8, then rotate step 1 will be anything from 0-45 deg, step 2 will be a random angle between 45 and 90, etc, etc
ttlIterations = len(ALPHABET_) * len(FONTS_) * ROTATE_STEPS_ * NUM_FONT_PTS_ * len(BLURS_)
pasteLayerWidth =  int(WIDTH_/1.5)
pasteLayerHeight = int(HEIGHT_/1.5)
currentPath = os.path.dirname(os.path.realpath(__file__))
pbar = tqdm(total=ttlIterations)

fontPtRange = FONT_PTS_[-1] - FONT_PTS_[0]
fontRangeStep = fontPtRange / NUM_FONT_PTS_

for font in FONTS_:
    for i in range(1, NUM_FONT_PTS_+1):
        font_pt_min = int(FONT_PTS_[0] + (i-1) * fontRangeStep)
        font_pt_max = int(FONT_PTS_[0] + i * fontRangeStep)
        fontFirstName = font.split(" ")[0].split(".")[0] # cleanup the font name so that we have something to make a unique image with
        for letter in ALPHABET_:
            for angleIter in range(1, ROTATE_STEPS_+1):
                for blur in BLURS_:

                    # do a different font pt size within our current font range for each image
                    font_pt = randint(font_pt_min, font_pt_max)
                    imgFnt = ImageFont.truetype(os.path.join(fontBasePath, font), font_pt)

                    savePath = os.path.join(BASE_PATH_, letter)
                    if not os.path.exists(os.path.join(currentPath, savePath)):
                        os.makedirs(os.path.join(currentPath, savePath))
                    
                    letterImg = Image.new('RGBA', (pasteLayerWidth, pasteLayerHeight))
                    drawn = ImageDraw.Draw(letterImg)
                    w, h = drawn.textsize(letter, font=imgFnt)
                    drawn.text(((pasteLayerWidth-w)/2,(pasteLayerHeight-h)/2), letter, fill=(0,0,0), font=imgFnt)
                    # rotate letter, get a random angle for the letter in the bounds of our current rotation angle
                    letterAngle = randint(rotateSection * (angleIter-1), rotateSection * angleIter)
                    letterImg = letterImg.rotate(letterAngle, expand=1)
                    if blur > 0:
                        letterImg = letterImg.filter(ImageFilter.GaussianBlur(radius=blur))
                    w, h = letterImg.size

                    baseImg = Image.new('RGB', (WIDTH_, HEIGHT_), color = (255,255,255))
                    baseImg.paste(letterImg, (int((WIDTH_ - w)/2) ,int((HEIGHT_-h)/2)), letterImg)
                    baseImg.save("{}{}{}-{}-{}blur{}-{}deg.jpeg".format(savePath, os.sep, letter, fontFirstName, font_pt, blur, letterAngle))
            pbar.update(ROTATE_STEPS_ * len(BLURS_))

pbar.close()
