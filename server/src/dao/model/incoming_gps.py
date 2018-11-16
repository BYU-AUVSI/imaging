class incoming_gps:

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getTime(self):
        return self.time

    def setTime(self, time):
        self.time = time

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
    time = property(getTime, setTime)
    lat = property(getLat, setLat)
    lon = property(getLon, setLon)
    alt = property(getAlt, setAlt)
