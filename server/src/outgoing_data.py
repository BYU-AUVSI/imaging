class outgoing_data:

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getType(self):
        return self.type

    def setType(self, type):
        self.type = type

    def getLatitude(self):
        return self.latitude

    def setLatitude(self, latitude):
        self.latitude = latitude

    def getLongitude(self):
        return self.longitude

    def setLongitude(self, longitude):
        self.longitude = longitude

    def getOrientation(self):
        return self.orientation

    def setOrientation(self, orientation):
        self.orientation = orientation

    def getShape(self):
        return self.shape

    def setShape(self, shape):
        self.shape = shape

    def getBackgroundColor(self):
        return self.background_color

    def setBackgroundColor(self, background_color):
        self.background_color = background_color

    def getAlphanumeric(self):
        return self.alphanumeric

    def setAlphanumeric(self, alphanumeric):
        self.alphanumeric = alphanumeric

    def getAlphanumericColor(self):
        return self.alphanumeric_color

    def setAlphanumericColor(self, alphanumeric_color):
        self.alphanumeric_color = alphanumeric_color

    def getAutonomous(self):
        return self.autonomous

    def setAutonomous(self, autonomous):
        self.autonomous = autonomous

    def getDescription(self):
        return self.description

    def setDescription(self, description):
        self.description = description

    def getCroppedImagePath(self):
        return self.cropped_image_path

    def setCroppedImagePath(self, cropped_image_path):
        self.cropped_image_path = cropped_image_path

    id = property(getId, setId)
    type = property(getType, setType)
    latitude = property(getLatitude, setLatitude)
    longitude = property(getLongitude, setLongitude)
    orientation = property(getOrientation, setOrientation)
    shape = property(getShape, setShape)
    background_color = property(getBackgroundColor, setBackgroundColor)
    alphanumeric = property(getAlphanumeric, setAlphanumeric)
    alphanumeric_color = property(getAlphanumericColor, setAlphanumericColor)
    autonomous = property(getAutonomous, setAutonomous)
    description = property(getDescription, setDescription)
    cropped_image_path = property(getCroppedImagePath, setCroppedImagePath)
