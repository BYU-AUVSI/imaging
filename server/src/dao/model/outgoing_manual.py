class outgoing_manual:

    def __init__(self, tableValues=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.type = tableValues[1]
            self.latitude = tableValues[2]
            self.longitude = tableValues[3]
            self.orientation = tableValues[4]
            self.shape = tableValues[5]
            self.background_color = tableValues[6]
            self.alphanumeric = tableValues[7]
            self.alphanumeric_color = tableValues[8]
            self.description = tableValues[9]
            self.cropped_path = tableValues[10]
            self.submitted = tableValues[11]

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        self._id = id

    @property
    def cropped_id(self):
        return self._cropped_id

    @cropped_id.setter
    def cropped_id(self, cropped_id):
        self._cropped_id = cropped_id

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, type):
        self._type = type

    @property
    def latitude(self):
        return self._latitude

    @latitude.setter
    def latitude(self, latitude):
        self._latitude = latitude

    @property
    def longitude(self):
        return self._longitude

    @longitude.setter
    def longitude(self, longitude):
        self._longitude = longitude

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, orientation):
        self._orientation = orientation

    @property
    def shape(self):
        return self._shape

    @shape.setter
    def shape(self, shape):
        self._shape = shape

    @property
    def background_color(self):
        return self._background_color

    @background_color.setter
    def background_color(self, background_color):
        self._background_color = background_color

    @property
    def alphanumeric(self):
        return self._alphanumeric

    @alphanumeric.setter
    def alphanumeric(self, alphanumeric):
        self._alphanumeric = alphanumeric

    @property
    def alphanumeric_color(self):
        return self._alphanumeric_color

    @alphanumeric_color.setter
    def alphanumeric_color(self, alphanumeric_color):
        self._alphanumeric_color = alphanumeric_color

    @property
    def description(self):
        return self._description

    @description.setter
    def description(self, description):
        self._description = description

    @property
    def cropped_path(self):
        return self._cropped_path

    @cropped_path.setter
    def cropped_path(self, cropped_path):
        self._cropped_path = cropped_path

    @property
    def submitted(self):
        return self._Submitted

    @submitted.setter
    def submitted(self, Submitted):
        self._Submitted = Submitted
