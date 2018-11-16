class incoming_state:
    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getTime(self):
        return self.time_stamp

    def setTime(self, time_stamp):
        self.time_stamp = time_stamp

    def getRoll(self):
        return self.roll

    def setRoll(self, roll):
        self.roll = roll

    def getPitch(self):
        return self.pitch

    def setPitch(self, pitch):
        self.pitch = pitch

    def getYaw(self):
        return self.yaw

    def setYaw(self, yaw):
        self.yaw = yaw

    id = property(getId, setId)
    time = property(getTime, setTime)
    roll = property(getRoll, setRoll)
    pitch = property(getPitch, setPitch)
    yaw = property(getYaw, setYaw)
