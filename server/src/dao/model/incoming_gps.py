class incoming_gps:

    def __init__(self, tableValues=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.time_stamp = tableValues[1]
            self.lat = tableValues[2]
            self.lon = tableValues[3]
            self.alt = tableValues[4]

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        self._id = id

    @property
    def time_stamp(self):
        return self._time_stamp

    @time_stamp.setter
    def time_stamp(self, time_stamp):
        self._time_stamp = time_stamp

    @property
    def lat(self):
        return self._lat

    @lat.setter
    def lat(self, lat):
        self._lat = lat

    @property
    def lon(self):
        return self._lon

    @lon.setter
    def lon(self, lon):
        self._lon = lon

    @property
    def alt(self):
        return self._alt

    @alt.setter
    def alt(self, alt):
        self._alt = alt

    def insertValues(self):
        return [self.time_stamp, self.lat, self.lon, self.alt]

    def toDict(self):
        dict = {}
        dict['id'] = self.id
        dict['time_stamp'] = self.time_stamp
        dict['latitude'] = self.lat
        dict['longitude'] = self.lon
        dict['altitude'] = self.alt
        return dict

    def __str__(self):
        return """IncomingGps:\n
            \tid: {}\n
            \ttime_stamp: {}\n
            \tlatitude: {}\n
            \tlongitude: {}\n
            \taltitude: {}""".format(self.id, self.time_stamp, self.lat, self.lon, self.alt)
