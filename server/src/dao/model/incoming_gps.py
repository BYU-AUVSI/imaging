class incoming_gps:

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getTimeStamp(self):
        return self.time_stamp

    def setTimeStamp(self, time_stamp):
        self.time_stamp = time_stamp

    def getLat(self):
        return self.lat

    def setLat(self, lat):
        self.lat = lat

    def getLon(self):
        return self.lon

    def setLon(self, lon):
        self.lon = lon

    def getAlt(self):
        return self.alt

    def setAlt(self, alt):
        self.alt = alt

    id = property(getId, setId)
    time_stamp = property(getTimeStamp, setTimeStamp)
    lat = property(getLat, setLat)
    lon = property(getLon, setLon)
    alt = property(getAlt, setAlt)
