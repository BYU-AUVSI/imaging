import time # for sleeping between geolocation pulls
import threading
from dao.incoming_gps_dao import IncomingGpsDAO
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
        
        # no we can run stuff
        # geo = Geolocation()
        while not self._should_shutdown:
            print("chillin in gps loop")
            time.sleep(1)

    def shutdown(self):
        self._should_shutdown = True
