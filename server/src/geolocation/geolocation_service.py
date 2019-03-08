import time # for sleeping between geolocation pulls
import threading
from geolocation.geolocation import targetGeolocation
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.model.incoming_gps import incoming_gps
from dao.incoming_image_dao import IncomingImageDAO
from dao.cropped_manual_dao import CroppedManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.outgoing_manual_dao import OutgoingManualDAO
from config import defaultConfigPath

class GeolocationThread(threading.Thread):

    def __init__(self):
        super(GeolocationThread, self).__init__()
        self._should_shutdown = False

    def run(self):

        # wait until we get our first gps coordinate
        haveGps = False
        print("waiting for gps")
        while not haveGps and not self._should_shutdown:
            # check for any entries by just getting all
            dao = IncomingGpsDAO(defaultConfigPath())
            results = dao.getAll()
            dao.close()
            if results is None:
                time.sleep(1)
            else:
                haveGps = True # we've got something!

        print("Have gps! Using the first coordinate as our ground station. Starting geolocation...")
        
        # get the coordinates from the first recorded gps measurement
        # this will be the coordinates we use for the groundstation
        # in geolocation
        dao = IncomingGpsDAO(defaultConfigPath())
        groundstationGps = dao.getFirst()
        dao.close()

        # now we can run stuff
        geo = targetGeolocation(groundstationGps.lat, groundstationGps.lon)
        while not self._should_shutdown:

            # lets deal with manual classifications in the queue
            dao = OutgoingManualDAO(defaultConfigPath())
            classifications = dao.getUnlocatedClassifications()
            dao.close()

            if classifications is not None:
                for classification in classifications:
                    # now get all the info about gps, state and crop
                    dao = CroppedManualDAO(defaultConfigPath())
                    croppedImg = dao.getImage(classification.crop_id)
                    dao.close()
                    if croppedImg is None:
                        print("Failed to find manual cropped image {} for manual classification {}!".format(classification.crop_id, classification.class_id))
                        continue

                    dao = IncomingImageDAO(defaultConfigPath())
                    rawImg = dao.getImage(croppedImg.image_id)
                    dao.close()
                    if rawImg is None:
                        print("Failed to find raw image {} for manual cropped image {}!".format(croppedImg.image_id, croppedImg.crop_id))
                        continue

                    dao = IncomingGpsDAO(defaultConfigPath())
                    gpsRaw = dao.getGpsByClosestTS(rawImg.time_stamp)
                    dao.close()

            time.sleep(1)

    def shutdown(self):
        self._should_shutdown = True
