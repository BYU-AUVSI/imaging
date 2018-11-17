class incoming_gps:

    @property
    def id(self):
        return self._id

    @id.setter
    def Id(self, id):
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
