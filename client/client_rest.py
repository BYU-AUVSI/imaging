import requests
import json
from PIL import Image
from io import BytesIO

class ImageInfo:
    def __init__(self, auto_tap, imageId, path, man_tap, ts):
        self.autonomous_tap = auto_tap
        self.id
        self.image_path = path
        self.manual_tap = man_tap
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
        self.ids = []
        self.idIndex = 0
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
        if self.idIndex >= -1:
            self.idIndex = -1
            img = requests.get(self.url + "/image/raw", headers={'X-Manual': str(isManual)})
            self.debug("response code:: {}".format(img.status_code))
            if (img.status_code != 200):
                # if we didnt get a good status code
                print("Server returned status code {}".format(img.status_code))
                return None

            imageId = int(img.headers['X-Image-Id'])
            if len(self.ids) >= self.numIdsStored:
                self.ids.pop(0)

            self.ids.append(imageId)
            self.debug("Image ID:: {}".format(imageId))
            return Image.open(BytesIO(img.content))
        else:
            self.idIndex += 1
            return self.getRawImage(self.ids[self.idIndex])

    def getPrevRawImage(self):
        """
        Re-retrive a raw image that you were previously viewing. This interface maintains an ordered list
        (of up to numIdsStored) of ids you've previously received and will traverse it backwards.

        @:rtype Image
        @:return A pillow Image if there are any previous images to process, and the server is able to find the given id, otherwise None.
        """
        self.debug("getPrevRawImage()")
        if len(self.ids) > 0:
            # if there is no more previous images, get the last image
            if abs(self.idIndex) >= len(self.ids):
                self.idIndex = -1 * len(self.ids)
            else: # else get the previous
                self.idIndex -= 1

            imageId = self.ids[self.idIndex]
            return self.getRawImage(imageId)
        else:
            self.debug("We haven't gotten any images yet")
            return 0

    def getImageInfo(self, imageId):
        self.debug("getImageInfo(id={})".format(imageId))
        img = requests.get(self.url + "/image/raw/" + str(imageId) + "/info")
        self.debug("response code:: {}".format(img.status_code))
        info_j = json.loads(img.content.decode('utf-8'))
        return ImageInfo(info_j['autonomous_tap'].lower() == 'true',
                         imageId,
                         info_j['image_path'],
                         info_j['manual_tap'].lower() == 'true',
                         float(info_j['time_stamp']))


    def getNextCroppedImage(self):
        return 0

    def getCroppedImage(self, imageId):
        return 0

    def getCroppedImageInfo(self, imageId):
        return 0

    def getAllCroppedInfo(self):
        return 0

    def postCroppedImage(self, imageId, image, tl, br):

        url = self.url + "/image/crop/"
        headers = {'X-Image_Id': str(imageId)}
        payload = {'crop_coordinate_tl': 'tl', 'crop_coordinate_br': 'br'}
        r = requests.post(url, data=json.dumps(), headers=headers)
        # image object
        # crop_coordinate_tl "(x, y)"
        # crop_coordinate_br "(x, y)"
        return 0

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

def testNextAndPrevImage():
    # interface = ImagingInterface(numIdsStored=4)
    interface = ImagingInterface(numIdsStored=4)


    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)

    print(interface.ids) # Check if it's popping the correct id

    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()
    interface.getPrevRawImage()

    print(interface.ids) # Check if it's continually getting the last id

    interface.getNextRawImage(True) # Check if it gets old ids then new ones as well
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)
    interface.getNextRawImage(True)

    print(interface.ids)
    print(interface.idIndex)
    print(interface.ids[interface.idIndex])


if __name__ == "__main__":
    # testNextAndPrevImage()
    interface = ImagingInterface()

    info = interface.getImageInfo(2)
    print(info.image_path)
