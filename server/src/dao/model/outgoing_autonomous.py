class outgoing_autonomous:
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

    def getBackground_color(self):
        return self.background_color

    def setBackground_color(self, background_color):
        self.background_color = background_color

    def getAlphanumeric(self):
        return self.alphanumeric

    def setAlphanumeric(self, alphanumeric):
        self.alphanumeric = alphanumeric

    def getAutonomous_color(self):
        return self.alphanumeric_color

    def setAutonomous_color(self, alphanumeric_color):
        self.alphanumeric_color = alphanumeric_color

    def getDescription(self):
        return self.description

    def setDescription(self, description):
        self.description = description

    def getCropped_path(self):
        return self.cropped_path

    def setCropped_path(self, cropped_path):
        self.cropped_path = cropped_path

    def getSubmitted(self):
        return self.Submitted

    def setSubmitted(self, Submitted):
        self.Submitted = Submitted


    id = property(getId, setId)
    type = property(getType, setType)
    latitude = property(getLatitude, setLatitude)
    longitude = property(getLongitude, setLongitude)
    orientation = property(getOrientation, setOrientation)
    shape = property(getShape, setShape)
    background_color = property(getBackground_color, setBackground_color)
    alphanumeric = property(getAlphanumeric, setAlphanumeric)
    alphanumeric_color = property(getAutonomous_color, setAutonomous_color)
    description = property(getDescription, setDescription)
    cropped_path = property(getCropped_path, setCropped_path)
    Submitted = property(getSubmitted, setSubmitted)
