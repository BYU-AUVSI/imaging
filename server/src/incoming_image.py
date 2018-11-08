class incoming_image:

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

    def getImagePath(self):
        return self.image_path

    def setImagePath(self, image_path):
        self.image_path = image_path

id = property(getId, setId)
time_stamp = property(getTimeStamp, setTimeStamp)
nanoseconds = property(getNanoseconds, setNanoseconds)
image_path = property(getImagePath, setImagePath)
