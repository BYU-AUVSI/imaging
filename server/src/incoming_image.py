class IncomingImage:

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getTime(self):
        return self.time_stamp

    def setTime(self, time_stamp):
        self.time_stamp = time_stamp

    def getImagePath(self):
        return self.image_path

    def setImagePath(self, image_path):
        self.image_path = image_path

    def getClaimedManual(self):
        return self.claimed_manual

    def setClaimedManual(self, claimed_manual):
        self.claimed_manual = claimed_manual

    def getClaimedAutonomous(self):
        return self.claimed_autonomous

    def setClaimedAutonomous(self, claimed_autonomous):
        self.claimed_autonomous = claimed_autonomous

id = property(getId, setId)
time = property(getTime, setTime)
image_path = property(getImagePath, setImagePath)
claimed_manual = property(getClaimedManual, setClaimedManual)
claimed_autonomous = property(getClaimedAutonomous, setClaimedAutonomous)
