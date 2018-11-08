class incoming_state:

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getTimeStamp(self):
        return self.time_stamp

    def setTimeStamp(self, time_stamp):
        self.time_stamp = time_stamp

    def getNanoseconds(self):
        return self.nanoseconds

    def setNanoseconds(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def getRoll(self):
        return self.roll

    def.setRoll(self, roll):
        self.roll = roll

    def getPitch(self):
        return self.pitch

    def.setPitch(self, pitch):
        self.pitch = pitch

    def getYaw(self):
        return self.yaw

    def.setYaw(self, yaw):
        self.yaw = yaw

    id = property(getId, setId)
    time_stamp = property(getTimeStamp, setTimeStamp)
    nanoseconds = property(getNanoseconds, setNanoseconds)
    roll = property(getRoll, setRoll)
    pitch = property(getPitch, setPitch)
    yaw = property(getYaw, setYaw)
