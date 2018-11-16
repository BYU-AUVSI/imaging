class IncomingImage:

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getTimeStamp(self):
        return self.time_stamp

    def setTimeStamp(self, time_stamp):
        self.time_stamp = time_stamp

    def getImagePath(self):
        return self.image_path

    def setImagePath(self, image_path):
        self.image_path = image_path

    def getManualTapped(self):
        return self.manual_tapped

    def setManualTapped(self, manual_tapped):
        self.manual_tapped = manual_tapped

    def getAutonomousTapped(self):
        return self.autonomous_tapped

    def setAutonomousTapped(self, autonomous_tapped):
        self.autonomous_tapped = autonomous_tapped

    id = property(getId, setId)
    time_stamp = property(getTimeStamp, setTimeStamp)
    image_path = property(getImagePath, setImagePath)
    manual_tapped = property(getManualTapped, setManualTapped)
    autonomous_tapped = property(getAutonomousTapped, setAutonomousTapped)
