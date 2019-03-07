import time # for sleeping between geolocation pulls
import threading
from geolocation import targetGeolocation
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.model.incoming_gps import incoming_gps
from dao.classification_dao import ClassificationDAO
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
            dao = ClassificationDAO(defaultConfigPath())
            classifications = dao.getUnlocatedClassifications()
            dao.close()

            if classifications is not None:
                for classification in classifications:
                    # now get all the info about gps, state and crop
                    dao = IncomingGpsDAO(defaultConfigPath())
            time.sleep(1)

    def shutdown(self):
        self._should_shutdown = True
