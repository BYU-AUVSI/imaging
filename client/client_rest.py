import requests
import json
from PIL import Image
from io import BytesIO

class ImageInfo:
    def __init__(self, auto_tap, imageId, path, man_tap, ts):
        self.autonomous_tap = auto_tap
        self.id = imageId
        self.image_path = path
        self.manual_tap = man_tap
        self.time_stamp = ts

class CropInfo:
    def __init__(self, cropId, imgId, tl, br, path, isTapped, ts):
        self.cropId = cropId
        self.imgId = imgId
        self.tl = tl
        self.br = br
        self.path = path
        self.isTapped = isTapped
        self.time_stamp = ts

class GPSMeasurement:
    def __init__(self, alt, gpsId, lat, long, ts):
        self.altitude = alt
        self.id = gpsId
        self.latitude = lat
        self.longitude = long
        self.time_stamp = ts

class StateMeasurement:
    def __init__(self, stateId, roll, pitch, yaw, ts):
        self.id = stateId
        self.roll = roll
        self. pitch = pitch
        self.yaw = yaw
        self.time_stamp = ts


class ImagingInterface:

    def __init__(self,
                 host="127.0.0.1",
                 port="5000",
                 numIdsStored=50,
                 isDebug=False):
        self.host = host
        self.port = port
        self.url = "http://" + self.host + ":" + self.port
        self.rawIds = []
        self.cropIds = []
        self.rawIdIndex = 0
        self.cropIdIndex = 0
        self.numIdsStored = numIdsStored
        self.isDebug = isDebug

    def debug(self, printStr):
        if self.isDebug:
            print(printStr)

    def getRawImage(self, imageId):
        self.debug("getImage(id={})".format(imageId))
        img = requests.get(self.url + "/image/raw/" + str(imageId), headers={'X-Manual': 'True'})
        self.debug("response code:: {}".format(img.status_code))
        return Image.open(BytesIO(img.content))

    def getNextRawImage(self, isManual):
        """
        Retrieve the next available raw image.

        @:type isManual: boolean
        @:param isManual: Specify whether this is a manual imaging request (True) or an autonomous one (False)

        @:rtype Image
        @:return A pillow Image if there are any images available for processing, otherwise None
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
            return Image.open(BytesIO(img.content))
        else:
            self.rawIdIndex += 1
            return self.getRawImage(self.rawIds[self.rawIdIndex])

    def getPrevRawImage(self):
        """
        Re-retrive a raw image that you were previously viewing. This interface maintains an ordered list
        (of up to numIdsStored) of ids you've previously received and will traverse it backwards.

        @:rtype Image
        @:return A pillow Image if there are any previous images to process, and the server is able to find the given id, otherwise None.
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
            return 0

    def getImageInfo(self, imageId):
        self.debug("getImageInfo(id={})".format(imageId))
        imgInfoResp = requests.get(self.url + "/image/raw/" + str(imageId) + "/info")
        self.debug("response code:: {}".format(imgInfoResp.status_code))
        info_j = json.loads(imgInfoResp.content.decode('utf-8'))
        return ImageInfo(info_j['autonomous_tap'].lower() == 'true',
                         imageId,
                         info_j['image_path'],
                         info_j['manual_tap'].lower() == 'true',
                         float(info_j['time_stamp']))


    def getCroppedImage(self, imageId):
        self.debug("getCroppedImage(id={})".format(imageId))
        img = requests.get(self.url + "/image/crop/" + str(imageId), headers={'X-Manual': 'True'})
        self.debug("response code:: {}".format(img.status_code))
        if img.status_code != 200:
            # if we didnt get a good status code
            print("Server returned status code {}".format(img.status_code))
            return None
        return Image.open(BytesIO(img.content))

    def getNextCroppedImage(self):
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
            return Image.open(BytesIO(img.content))
        else:
            self.cropIdIndex += 1
            return self.getCroppedImage(self.cropIds[self.cropIdIndex])

    def getPrevCroppedImage(self):
        """
        Re-retrive a cropped image that you were previously viewing. This interface maintains an ordered list
        (of up to numIdsStored) of ids you've previously received and will traverse it backwards.

        @:rtype Image
        @:return A pillow Image if there are any previous images to process, and the server is able to find the given id, otherwise None.
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
        self.debug("getCroppedImageInfo(id={})".format(imageId))
        cropInfoResp = requests.get(self.url + "/image/crop/" + str(imageId) + "/info")
        self.debug("response code:: {}".format(cropInfoResp.status_code))
        info_j = json.loads(cropInfoResp.content.decode('utf-8'))
        # tl = info_j['crop_coordinate_tl']
        # br = info_j['crop_coordinate_br']
        return CropInfo(int(info_j['id']),
                        imageId,
                        [info_j['crop_coordinate_tl.x'], info_j['crop_coordinate_tl.y']],
                        [info_j['crop_coordinate_br.x'], info_j['crop_coordinate_br.y']],
                        info_j['cropped_path'],
                        info_j['tapped'].lower() == 'true',
                        float(info_j['time_stamp']))

    def getAllCroppedInfo(self):
        self.debug("getAllCroppedInfo")
        resp = requests.get(self.url + "/image/crop/all")
        if resp.status_code != 200:
            # if we didnt get a good status code
            print("Server returned status code {}".format(resp.status_code))
            return None
        cropInfoList = []
        cropInfoList_j = json.loads(resp.content.decode('utf-8'))
        for i in range(len(cropInfoList_j)):
            # cropInfoList.append(self.getCroppedImageInfo(int(cropInfoList_j[i]['image_id'])))
            cropInfoList.append(CropInfo(None,
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
        imgByteArr = BytesIO()
        img.save(imgByteArr, format='JPEG')
        imgByteArr = imgByteArr.getvalue()
        return imgByteArr


    def postCroppedImage(self, imageId, crop, tl, br):
        """
            Description: Posts a cropped image to the server

            @:type  imageId: integer
            @:param imageId: The id to the original image being cropped

            @:type  crop: PIL Image
            @:param crop: The image file of the cropped image

            @:type  tl: Integer array of length 2
            @:param tl: The x and y coordinate of the location of the cropped image
                        in the top left corner relative to the original image

            @:type  br: Integer array of length 2
            @:param br: The x and y coordinate of the location of the cropped image
                        in the bottom right corner relative to the original image

            @:rtype Response
            @:return The response of the http request
        """
        self.debug("postCroppedImage(imageId={})".format(imageId))
        url = self.url + "/image/crop/"
        headers = {'X-Image_Id': str(imageId)}
        tlStr = "(" + str(tl[0]) + ", " + str(tl[1]) + ")"
        brStr = "(" + str(br[0]) + ", " + str(br[1]) + ")"

        data = {'crop_coordinate_tl': tlStr, 'crop_coordinate_br': brStr}
        files = {'cropped_image': self.imageToBytes(crop)}
        resp = requests.post(url, data=data, headers=headers, files=files)
        return resp

    def getGPSByTs(self, ts):
        self.debug("getGPSByTs(ts={})".format(ts))
        gps = requests.get(self.url + "/gps/ts/" + str(ts))
        self.debug("response code:: {}".format(gps.status_code))
        info_j = json.loads(gps.content.decode('utf-8'))
        return GPSMeasurement(info_j['altitude'],
                              info_j['id'],
                              info_j['latitude'],
                              info_j['longitude'],
                              info_j['time_stamp'])

    def getGPSById(self, gpsId):
        self.debug("getIGPSById(id={})".format(gpsId))
        gps = requests.get(self.url + "/gps/" + str(gpsId))
        self.debug("response code:: {}".format(gps.status_code))
        info_j = json.loads(gps.content.decode('utf-8'))
        return GPSMeasurement(info_j['altitude'],
                              info_j['id'],
                              info_j['latitude'],
                              info_j['longitude'],
                              info_j['time_stamp'])


    def getStateByTs(self, ts):
        self.debug("getStateByTs(ts={})".format(ts))
        state = requests.get(self.url + "/state/ts/" + str(ts))
        self.debug("response code:: {}".format(state.status_code))
        info_j = json.loads(state.content.decode('utf-8'))
        return StateMeasurement(info_j['id'],
                                info_j['roll'],
                                info_j['pitch'],
                                info_j['yaw'],
                                info_j['time_stamp'])

    def getStateById(self, stateId):
        self.debug("getIGPSById(id={})".format(stateId))
        state = requests.get(self.url + "/state/" + str(stateId))
        self.debug("response code:: {}".format(state.status_code))
        info_j = json.loads(state.content.decode('utf-8'))
        return StateMeasurement(info_j['id'],
                                info_j['roll'],
                                info_j['pitch'],
                                info_j['yaw'],
                                info_j['time_stamp'])

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

    print("Done")
