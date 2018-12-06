import requests
import json
from PIL import Image
from io import BytesIO

class ImageInfo:
    """
    An ImageInfo object holds the information concerning an image such as the image id, the image path,
    the timestamp the image was taken, and whether or not it has been seen by the autonomous or manual system.
    """
    def __init__(self, auto_tap, imageId, path, man_tap, ts):
        """
        The constructor for the ImageInfo class.

        @:type  auto_tap: boolean
        @:param auto_tap: True if the autonomous system has seen the image, False otherwise

        @:type  imageId: int
        @:param imageId: the id related to the image

        @:type  path: String
        @:param path: the file path to where the image is located

        @:type  man_tap: boolean
        @:param man_tap: True if the manual system has seen the image, False otherwise

        @:type  ts: float
        @:param ts: the timestamp that the image was taken
        """
        self.autonomous_tap = auto_tap
        self.id = imageId
        self.image_path = path
        self.manual_tap = man_tap
        self.time_stamp = ts

class CropInfo:
    """
    A CropInfo object holds information concerning a cropped image such as the image id to the original image,
    the top-left and bottom-right coordinates to where the crop took place on the original image,
    the path of the cropped image, whether or not the cropped image has been seen by the imaging system,
    and the timestamp of the original image.
    """
    def __init__(self, imgId, tl, br, path, isTapped, ts):
        """
        The constructor for the CropInfo class.

        @:type  imgId: int
        @:param imgId: the id of the original image

        @:type  tl: int[]
        @:param tl: the x and y coordinates of the original image of the top left corner to where the image was cropped

        @:type  br: int[]
        @:param br: the x and y coordinates of the original image of the bottom right corner to where the image was cropped

        @:type  path: String
        @:param path: the file path to where the cropped image is located

        @:type  isTapped: boolean
        @:param isTapped: True if the manual imaging system has seen this image, False otherwise

        @:type  ts: float
        @:param ts: the timestamp that the image was taken
        """
        self.imgId = imgId
        self.tl = tl
        self.br = br
        self.path = path
        self.isTapped = isTapped
        self.time_stamp = ts

class GPSMeasurement:
    """
    A GPSMeasurement object holds information concerning one measurement taken from the aircraft's GPS.
    Such information includes the id of the measurement, altitude in meters, longitude, latitude,
    and the timestamp of when the measurement was taken.
    """
    def __init__(self, alt, gpsId, lat, long, ts):
        """
        The constructor for the GPSMeasurement class.

        @:type  alt: float
        @:param alt: the altitude of the measurement

        @:type  gpsId: int
        @:param gpsId: the id of the GPS measurement

        @:type  lat: float
        @:param lat: the latitude of the measurement

        @:type  long: float
        @:param long: the longitude of the measurement

        @:type  ts: float
        @:param ts: the timestamp that the measurement was taken
        """
        self.altitude = alt
        self.id = gpsId
        self.latitude = lat
        self.longitude = long
        self.time_stamp = ts

class StateMeasurement:
    """
    A StateMeasurement object holds information concerning the state of the aircraft in one point in time.
    Such information includes the id of the measurement, roll angle of the aircraft in radians,
    pitch angle of the aircraft in radians, yaw angle of the aircraft in radians,
    and the timestamp of when the measurement was taken.
    """
    def __init__(self, stateId, roll, pitch, yaw, ts):
        """
        The constructor for the StateMeasurement class.

        @:type  stateId: int
        @:param stateId: the id of the state measurement

        @:type  roll: float
        @:param roll: the roll angle in radians of the aircraft at the time the measurement was taken

        @:type  pitch: float
        @:param pitch: the pitch angle in radians of the aircraft at the time the measurement was taken

        @:type  yaw: float
        @:param yaw: The yaw angle in radians of the aircraft at the time the measurement was taken

        @:type  ts: float
        @:param ts: the timestamp that the measurement was taken
        """
        self.id = stateId
        self.roll = roll
        self. pitch = pitch
        self.yaw = yaw
        self.time_stamp = ts


class ImagingInterface:
    """
    The ImagingInterface object serves as the bridge between the imaging server and the imaging GUI.
    It connects to the imaging server given an IP address and a port. It makes certain calls to the server
    in order to feed the GUI what it needs such as images and measurements taken by the imaging system on the aircraft.
    """
    def __init__(self,
                 host="127.0.0.1",
                 port="5000",
                 numIdsStored=50,
                 isDebug=False):
        """
        The constructor for the ImagingInterface class.

        @:type  host: String
        @:param host: the host of the server that the interface connects to

        @type  port: String
        @:param port: the port of the server that the interface connects to

        @:type  numIdsStored: int
        @:param numIdsStored: the number of ids that the interface keeps track of, used for getting previous images

        @:type  isDebug: boolean
        @:param isDebug: if isDebug is true, the interface will print out status statements

        @:rtype:  None
        @:return: None
        """
        self.host = host
        self.port = port
        self.url = "http://" + self.host + ":" + self.port
        self.rawIds = []
        self.cropIds = []
        self.rawIdIndex = 0
        self.cropIdIndex = 0
        self.numIdsStored = numIdsStored
        self.isDebug = isDebug

    def ping(self):
        """
        Checks to see if the interface can contact the server.

        @:rtype:  boolean
        @:return: True if the interface contacted the server, False otherwise
        """
        try:
            self.getGPSById(1)
        except:
            return False
        return True
        

    def debug(self, printStr):
        """
        If interface is in debug mode, it will print the string given, else it does nothing.

        @:type  printStr: String
        @:param printStr: the string that will be printed if in debug mode

        @:rtype:  None
        @:return: None
        """
        if self.isDebug:
            print(printStr)

    def getRawImage(self, imageId):
        """
        Retrieves an image with the given imageId from the server.

        @:type  imageId: int
        @:param imageId: the id of the image that is going to be returned

        @:rtype:  (Image, int)
        @:return: a tuple of the pillow Image associated with the given image id and the image id
        if there are any images available for processing, otherwise None
        """
        self.debug("getImage(id={})".format(imageId))
        img = requests.get(self.url + "/image/raw/" + str(imageId), headers={'X-Manual': 'True'})
        self.debug("response code:: {}".format(img.status_code))
        if img.status_code == 200:
            return Image.open(BytesIO(img.content)), imageId
        else:
            print("Server returned status code {}".format(img.status_code))
            return None

    def getNextRawImage(self, isManual):
        """
        Retrieves the next available raw image from the server.

        @:type  isManual: boolean
        @:param isManual: specify whether this is a manual imaging request (True) or an autonomous one (False)

        @:rtype:  (Image, int)
        @:return: a tuple of a pillow Image and the image id if there are any images available for processing,
        otherwise None
        """
        self.debug("getNextRawImage(isManual={})".format(isManual))
        if self.rawIdIndex >= -1:
            self.rawIdIndex = -1
            img = requests.get(self.url + "/image/raw/", headers={'X-Manual': str(isManual)})
            self.debug("response code:: {}".format(img.status_code))
            if img.status_code != 200:
                # if we didnt get a good status code
                print("Server returned status code {}".format(img.status_code))
                return None

            imageId = int(img.headers['X-Image-Id'])
            if len(self.rawIds) >= self.numIdsStored:
                self.rawIds.pop(0)

            self.rawIds.append(imageId)
            self.debug("Image ID:: {}".format(imageId))
            return Image.open(BytesIO(img.content)), imageId
        else:
            self.rawIdIndex += 1
            return self.getRawImage(self.rawIds[self.rawIdIndex])

    def getPrevRawImage(self):
        """
        Re-retrieves a raw image that was previously viewed. The interface maintains an ordered list
        (of up to numIdsStored) of ids that has previously been viewed and traverses the list backwards.

        @:rtype:  (Image, int)
        @:return: a tuple of a pillow Image and the image id if there are any previous images to process,
        and the server is able to find the given id, otherwise None.
        """
        self.debug("getPrevRawImage()")
        if len(self.rawIds) > 0:
            # if there is no more previous images, get the last image
            if abs(self.rawIdIndex) >= len(self.rawIds):
                self.rawIdIndex = -1 * len(self.rawIds)
            else: # else get the previous
                self.rawIdIndex -= 1

            imageId = self.rawIds[self.rawIdIndex]
            return self.getRawImage(imageId)
        else:
            self.debug("We haven't gotten any images yet")
            return None

    def getImageInfo(self, imageId):
        """
        Retrieves information about an image from the server given the image id.

        @:type  imageId: int
        @:param imageId: the id of the image of interest

        @:rtype:  ImageInfo
        @:return: an object that contains the information about the given image
        if it exists and connects to the server, otherwise None
        """
        self.debug("getImageInfo(id={})".format(imageId))
        imgInfoResp = requests.get(self.url + "/image/raw/" + str(imageId) + "/info")
        self.debug("response code:: {}".format(imgInfoResp.status_code))
        if imgInfoResp.status_code == 200:
            info_j = json.loads(imgInfoResp.content.decode('utf-8'))
            return ImageInfo(info_j['autonomous_tap'].lower() == 'true',
                            imageId,
                            info_j['image_path'],
                            info_j['manual_tap'].lower() == 'true',
                            float(info_j['time_stamp']))
        else:
            print("Server returned status code {}".format(imgInfoResp.status_code))
            return None

    def getCroppedImage(self, imageId):
        """
        Retrieves a cropped image of the image from the server given the imageId.

        @:type  imageId: int
        @:param imageId: the id of the image

        @:rtype:  (Image, int)
        @:return: a tuple of a pillow Image and the image id if the image with that id is cropped, otherwise None
        """
        self.debug("getCroppedImage(id={})".format(imageId))
        img = requests.get(self.url + "/image/crop/" + str(imageId), headers={'X-Manual': 'True'})
        self.debug("response code:: {}".format(img.status_code))
        if img.status_code != 200:
            # if we didnt get a good status code
            print("Server returned status code {}".format(img.status_code))
            return None
        return Image.open(BytesIO(img.content)), imageId

    def getNextCroppedImage(self):
        """
        Retrieves the next available cropped image from the server.

        @:rtype:  (Image, int)
        @:return: a tuple of a pillow Image and the image id if the image with that id is cropped, otherwise None
        """
        self.debug("getNextCroppedImage()")
        if self.cropIdIndex >= -1:
            self.cropIdIndex = -1
            img = requests.get(self.url + "/image/crop/")
            self.debug("response code:: {}".format(img.status_code))
            if img.status_code != 200:
                # if we didnt get a good status code
                print("Server returned status code {}".format(img.status_code))
                return None

            imageId = int(img.headers['X-Image-Id'])
            if len(self.cropIds) >= self.numIdsStored:
                self.cropIds.pop(0)

            self.cropIds.append(imageId)
            self.debug("Image ID:: {}".format(imageId))
            return Image.open(BytesIO(img.content)), imageId
        else:
            self.cropIdIndex += 1
            return self.getCroppedImage(self.cropIds[self.cropIdIndex])

    def getPrevCroppedImage(self):
        """
        Re-retrieves a cropped image that was previously viewed. The interface maintains an ordered list
        (of up to numIdsStored) of ids that has previously been viewed and traverses the list backwards.

        @:rtype:  (Image, int)
        @:return: a pillow Image if there are any previous images to process, and the server is able to find the given id,
        otherwise None.
        """
        self.debug("getPrevCroppedImage()")
        if len(self.cropIds) > 0:
            # if there is no more previous images, get the last image
            if abs(self.cropIdIndex) >= len(self.cropIds):
                self.cropIdIndex = -1 * len(self.cropIds)
            else: # else get the previous
                self.cropIdIndex -= 1

            imageId = self.cropIds[self.cropIdIndex]
            return self.getCroppedImage(imageId)
        else:
            self.debug("We haven't gotten any images yet")
            return None

    def getCroppedImageInfo(self, imageId):
        """
        Retrieves information about a cropped image from the server given the image id.

        @:type  imageId: int
        @:param imageId: the id of the image of interest

        @:rtype:  CropInfo
        @:return: an object that contains the information about the given cropped image
        if it exists and connects to the server, otherwise None
        """
        self.debug("getCroppedImageInfo(id={})".format(imageId))
        cropInfoResp = requests.get(self.url + "/image/crop/" + str(imageId) + "/info")
        if cropInfoResp.status_code == 200:
            self.debug("response code:: {}".format(cropInfoResp.status_code))
            info_j = json.loads(cropInfoResp.content.decode('utf-8'))
            # tl = info_j['crop_coordinate_tl']
            # br = info_j['crop_coordinate_br']
            return CropInfo(imageId,
                            [info_j['crop_coordinate_tl.x'], info_j['crop_coordinate_tl.y']],
                            [info_j['crop_coordinate_br.x'], info_j['crop_coordinate_br.y']],
                            info_j['cropped_path'],
                            info_j['tapped'].lower() == 'true',
                            float(info_j['time_stamp']))
        else:
            print("Server returned status code {}".format(cropInfoResp.status_code))
            return None

    def getAllCroppedInfo(self):
        """
        Retrieves the information pertaining to all of the cropped images in the server.

        @:rtype:  CropInfo[]
        @:return: a list of CropInfo objects of all of the cropped images if it connects to the server, otherwise None
        """
        self.debug("getAllCroppedInfo")
        resp = requests.get(self.url + "/image/crop/all")
        if resp.status_code != 200:
            # if we didnt get a good status code
            print("Server returned status code {}".format(resp.status_code))
            return None
        cropInfoList = []
        cropInfoList_j = json.loads(resp.content.decode('utf-8'))
        for i in range(len(cropInfoList_j)):
            cropInfoList.append(CropInfo(
                                int(cropInfoList_j[i]['image_id']),
                                [cropInfoList_j[i]['crop_coordinate_tl.x'],
                                cropInfoList_j[i]['crop_coordinate_tl.y']],
                                [cropInfoList_j[i]['crop_coordinate_br.x'],
                                cropInfoList_j[i]['crop_coordinate_br.y']],
                                cropInfoList_j[i]['cropped_path'],
                                cropInfoList_j[i]['tapped'].lower() == 'true',
                                float(cropInfoList_j[i]['time_stamp'])))
        return cropInfoList


    def imageToBytes(self, img):
        """
        Takes an Image object and returns the bytes of the given image.

        @:type  img: Image
        @:param img: the image to convert into bytes

        @:rtype:  bytes
        @:return: bytes of the given image
        """
        imgByteArr = BytesIO()
        img.save(imgByteArr, format='JPEG')
        imgByteArr = imgByteArr.getvalue()
        return imgByteArr

    def postCroppedImage(self, imageId, crop, tl, br):
        """
        Posts a cropped image to the server.

        @:type  imageId: integer
        @:param imageId: the id to the original image being cropped

        @:type  crop: PIL Image
        @:param crop: the image file of the cropped image

        @:type  tl: integer array of length 2
        @:param tl: the x and y coordinate of the location of the cropped image
                    in the top left corner relative to the original image

        @:type  br: integer array of length 2
        @:param br: the x and y coordinate of the location of the cropped image
                    in the bottom right corner relative to the original image

        @:rtype Response
        @:return The response of the http request if it successfully posts, otherwise None
        """
        self.debug("postCroppedImage(imageId={})".format(imageId))
        url = self.url + "/image/crop/"
        headers = {'X-Image_Id': str(imageId)}
        tlStr = "(" + str(tl[0]) + ", " + str(tl[1]) + ")"
        brStr = "(" + str(br[0]) + ", " + str(br[1]) + ")"

        data = {'crop_coordinate_tl': tlStr, 'crop_coordinate_br': brStr}
        files = {'cropped_image': self.imageToBytes(crop)}
        resp = requests.post(url, data=data, headers=headers, files=files)
        if resp.status_code == 200:
            return resp
        else:
            print("Server returned status code {}".format(resp.status_code))
            return None

    def getGPSByTs(self, ts):
        """
        Retrieves from the server the GPS measurement that is closest to the given timestamp.

        @:type  ts: float
        @:param ts: the timestamp of interest

        @:rtype:  GPSMeasurement
        @:return: GPSMeasurement object closest to the given timestamp if it connects to the server, otherwise None
        """
        self.debug("getGPSByTs(ts={})".format(ts))
        gps = requests.get(self.url + "/gps/ts/" + str(ts))
        self.debug("response code:: {}".format(gps.status_code))
        if gps.status_code == 200:
            info_j = json.loads(gps.content.decode('utf-8'))
            return GPSMeasurement(info_j['altitude'],
                                info_j['id'],
                                info_j['latitude'],
                                info_j['longitude'],
                                info_j['time_stamp'])
        else:
            print("Server returned status code {}".format(gps.status_code))
            return None

    def getGPSById(self, gpsId):
        """
        Retrieves from the server a GPS measurement given an id.

        @:type  gpsId: int
        @:param gpsId: the id of the gps measurement of interest

        @:rtype:  GPSMeasurement
        @:return: GPSMeasurement object of the given Id if it exists and connects to the server, otherwise None
        """
        self.debug("getIGPSById(id={})".format(gpsId))
        gps = requests.get(self.url + "/gps/" + str(gpsId), timeout=5)
        self.debug("response code:: {}".format(gps.status_code))
        if gps.status_code == 200:
            info_j = json.loads(gps.content.decode('utf-8'))
            return GPSMeasurement(info_j['altitude'],
                                info_j['id'],
                                info_j['latitude'],
                                info_j['longitude'],
                                info_j['time_stamp'])
        else:
            print("Server returned status code {}".format(gps.status_code))
            return None

    def getStateByTs(self, ts):
        """
        Retrieves from the server the state measurement that is closest to the given timestamp.


        @:type  ts: float
        @:param ts: the timestamp of interest

        @:rtype:  StateMeasurement
        @:return: StateMeasurement object closest to the given timestamp if it connects to the server, otherwise None
        """
        self.debug("getStateByTs(ts={})".format(ts))
        state = requests.get(self.url + "/state/ts/" + str(ts))
        self.debug("response code:: {}".format(state.status_code))
        if state.status_code == 200:            
            info_j = json.loads(state.content.decode('utf-8'))
            return StateMeasurement(info_j['id'],
                                    info_j['roll'],
                                    info_j['pitch'],
                                    info_j['yaw'],
                                    info_j['time_stamp'])
        else:
            print("Server returned status code {}".format(state.status_code))
            return None

    def getStateById(self, stateId):
        """
        Retrieves from the server a state measurement given an id.

        @:type  stateId: int
        @:param stateId: the id of the state measurement of interest

        @:rtype:  StateMeasurement
        @:return: StateMeasurement object of the given Id if it exists and connects to the server, otherwise None
        """
        self.debug("getIGPSById(id={})".format(stateId))
        state = requests.get(self.url + "/state/" + str(stateId))
        self.debug("response code:: {}".format(state.status_code))
        if state.status_code == 200:
            info_j = json.loads(state.content.decode('utf-8'))
            return StateMeasurement(info_j['id'],
                                    info_j['roll'],
                                    info_j['pitch'],
                                    info_j['yaw'],
                                    info_j['time_stamp'])
        else:
            print("Server returned status code {}".format(state.status_code))
            return None

def testNextAndPrevRawImage(interface):
    interface.numIdsStored = 4

    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)

    print(interface.rawIds) # Check if it's popping the correct id

    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()

    print(interface.rawIds) # Check if it's continually getting the last id

    interface.getNextRawImage(True) # Check if it gets old ids then new ones as well
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)

    print(interface.rawIds)
    print(interface.rawIdIndex)
    print(interface.rawIds[interface.rawIdIndex])


def testNextAndPrevCroppedImage(interface):
    interface.numIdsStored = 4

    interface.getNextCroppedImage()
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()

    print(interface.cropIds)  # Check if it's popping the correct id

    interface.getPrevCroppedImage()
    interface.getPrevCroppedImage()
    interface.getPrevCroppedImage()
    interface.getPrevCroppedImage()
    interface.getPrevCroppedImage()
    interface.getPrevCroppedImage()

    print(interface.cropIds)  # Check if it's continually getting the last id

    interface.getNextCroppedImage()  # Check if it gets old ids then new ones as well
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()
    interface.getNextCroppedImage()

    print(interface.cropIds)
    print(interface.cropIdIndex)
    print(interface.cropIds[interface.cropIdIndex])


def testCropPost(interface, imgId):
    img = interface.getRawImage(imgId)
    resp = interface.postCroppedImage(imgId, img, [0, 0], [236, 236])
    print(resp.status_code)
    print(resp.text)
    return resp



if __name__ == "__main__":
    interface = ImagingInterface(host="127.0.0.1", isDebug=True)
    # interface = ImagingInterface(host="192.168.1.48", isDebug=True)
    # imgId = 2
    infoList = interface.getAllCroppedInfo()
    query = interface.getRawImage(5)

    print("Done")
