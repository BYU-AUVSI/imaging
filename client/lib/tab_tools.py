import cv2
from PIL import Image,ImageTk
import numpy as np

def get_image(path):
    """
    Reads in an image from folder on computer

    @type  path: file path
    @param path: the file path to where the image is located

    @rtype:  Numpy image array
    @return: Numpy array of selected image
    """
    image_np = cv2.imread(path)
    image_np = cv2.cvtColor(image_np,cv2.COLOR_BGR2RGB)
    return image_np

def np2im(image):
    """
    Converts from numpy array to PIL image
    @type image: Numpy image array
    @param image: Numpy array of selected image

    @rtype:  PIL image
    @return: PIL image of numpy array
    """
    image_im = Image.fromarray(image)
    return image_im

def im2tk(image):
    """
    Converts from PIL image to TK image
    @type image: PIL image
    @param image: PIL image of numpy array

    @rtype:  TK image
    @return: TK image of PIL image
    """
    image_tk = ImageTk.PhotoImage(image)
    return image_tk

def resizeIm(image,image_width,image_height,width_restrict,height_restrict):
    """
    Resizes PIL image according to given bounds
    @type  image: PIL image
    @param image: PIL image that you want to crop

    @type  image_width: integer
    @param image_width: the original image width in pixels

    @type  image_height: integer
    @param image_height: the original image height in pixels

    @type  width_restrict: integer
    @param width_restrict: the width in pixels of restricted area

    @type  height_restrict: integer
    @param height_restrict: the height in pixels of restricted area

    @rtype:  PIL image
    @return: Resized PIL image
    """
    ratio_h = height_restrict/image_height
    ratio_w = width_restrict/image_width
    if ratio_h <= ratio_w:
        resized_im = image.resize((int(image_width*ratio_h), int(image_height*ratio_h)), Image.ANTIALIAS)
    else:
        resized_im = image.resize((int(image_width*ratio_w), int(image_height*ratio_w)), Image.ANTIALIAS)
    return(resized_im)

def getYawAngle(interface,imageID):
    info = interface.getCroppedImageInfo(imageID)
    image_state = None
    if info is not None:
        image_state = interface.getStateByTs(info.time_stamp)
    if image_state == None:
        #yaw_angle = 0.0
        # todo: eventually remove this
        yaw = np.random.uniform(0,360)
    else:
        yaw = image_state.yaw
    return(yaw)