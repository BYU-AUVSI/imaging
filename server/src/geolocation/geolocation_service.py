import time # for sleeping between geolocation pulls
import threading
from geolocation.geolocation import targetGeolocation
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.incoming_state_dao import IncomingStateDAO
from dao.model.incoming_gps import incoming_gps
from dao.incoming_image_dao import IncomingImageDAO
from dao.cropped_manual_dao import CroppedManualDAO
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.cropped_autonomous_dao import CroppedAutonomousDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from config import defaultConfigPath

class GeolocationThread(threading.Thread):

    def __init__(self):
        super(GeolocationThread, self).__init__()
        self._should_shutdown = False

    def processGeolocation(self, geolocator, classification, croppedImg):
        """
        Geolocation processing stuff common to both manual and autonomous.
        Called only by the processManualGeolocation and processAutonomousGeolocation
        methods
        """
        dao = IncomingImageDAO(defaultConfigPath())
        rawImg = dao.getImage(croppedImg.image_id)
        dao.close()
        if rawImg is None:
            print("Failed to find raw image {} for autonomous cropped image {}!".format(croppedImg.image_id, croppedImg.crop_id))
            return None

        dao = IncomingGpsDAO(defaultConfigPath())
        gpsRaw = dao.getGpsByClosestTS(rawImg.time_stamp)
        dao.close()

        if gpsRaw is None:
            print("Failed to find gps measurement close to raw timestamp {}!".format(rawImg.time_stamp))
            return None

        dao = IncomingStateDAO(defaultConfigPath())
        stateRaw = dao.getStateByClosestTS(rawImg.time_stamp)
        dao.close()

        if stateRaw is None:
            print("Failed to find state measurement close to raw timestamp {}!".format(rawImg.time_stamp))
            return None

        lat, lon = geolocator.calculate_geolocation(gpsRaw.lat, gpsRaw.lon, gpsRaw.alt, stateRaw.roll, stateRaw.pitch, stateRaw.yaw, croppedImg.crop_coordinate_tl.x, croppedImg.crop_coordinate_tl.y, croppedImg.crop_coordinate_br.x, croppedImg.crop_coordinate_br.y)

        return {'latitude': lat, 'longitude': lon}

    def processManualGeolocation(self, geolocator, classification):
        """
        Process an unlocated manual classification waiting in the queue. Basically
        this involves getting a bunch of information from a lot of the other database
        tables, and then using it as input to the actual geolocation algorithm.
        If successful, this will update the given classification with the calculated
        coordinates. Here's what we use:

            cropped_manual -> image_id, crop_coordinates
            incoming_image -> image timestamp
            incoming_gps   -> mav gps coordinates at incoming_image timestamp
            incoming_gps   -> mav state info at incoming_image timestamp

        @type geolocator: targetGeolocation
        @param geolocator: An instance of the targetGeolocation class, already 
            initialized
        
        @type classification: outgoing_manual
        @param classification: A manual classification that has no geolocation 
            coordinates yet.
        """

        # now get all the info about gps, state and crop
        # as we do this we need to check that we actually get
        # data back from our queries. if we dont, then just
        # skip this classification
        dao = CroppedManualDAO(defaultConfigPath())
        croppedImg = dao.getImage(classification.crop_id)
        dao.close()
        if croppedImg is None:
            print("Failed to find cropped image {} for manual classification {}!".format(classification.crop_id, classification.class_id))
            return

        updateDict = self.processGeolocation(geolocator, classification, croppedImg)

        dao = OutgoingManualDAO(defaultConfigPath())
        dao.updateClassification(classification.class_id, updateDict)
        dao.close()

    def processAutonomousGeolocation(self, geolocator, classification):
        """
        See documentation for processManualGeolocation function. This is the same
        thing but autonomous.
        """
        dao = CroppedAutonomousDAO(defaultConfigPath())
        croppedImg = dao.getImage(classification.crop_id)
        dao.close()
        if croppedImg is None:
            print("Failed to find cropped image {} for autonomous classification {}!".format(classification.crop_id, classification.class_id))
            return

        updateDict = self.processGeolocation(geolocator, classification, croppedImg)

        dao = OutgoingAutonomousDAO(defaultConfigPath())
        dao.updateClassification(classification.class_id, updateDict)
        dao.close()

    def run(self):

        # wait until we get our first gps coordinate
        # remember - the ros_handler code does not insert a gps record
        #           into the table until the gps has a fix
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
                    self.processManualGeolocation(geo, classification)

            # now lets do autonomous
            dao = OutgoingAutonomousDAO(defaultConfigPath())
            classifications = dao.getUnlocatedClassifications()
            dao.close()

            if classifications is not None:
                for classification in classifications:
                    self.processAutonomousGeolocation(geo, classification)

            time.sleep(1)

    def shutdown(self):
        self._should_shutdown = True
