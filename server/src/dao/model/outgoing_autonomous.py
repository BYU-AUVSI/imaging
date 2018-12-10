class outgoing_autonomous:
    """
    Model class for the autonomous classification 'outgoing_autonomous' table. This model
    is very similar to the outgoing_manual model class.
    """

    def __init__(self, tableValues=None, json=None):
        """
        Accepts various formats to instantiate this model object

        @type tableValues: [object]
        @param tableValues: List of table values, in table column order

        @type json: {object}
        @param json: Json dictionary of table values. Used by the REST API when receiving data
        """
        if tableValues is not None:
            self.id = tableValues[0]
            self.image_id = tableValues[1]
            self.type = tableValues[2]
            self.latitude = tableValues[3]
            self.longitude = tableValues[4]
            self.orientation = tableValues[5]
            self.shape = tableValues[6]
            self.background_color = tableValues[7]
            self.alphanumeric = tableValues[8]
            self.alphanumeric_color = tableValues[9]
            self.description = tableValues[10]
            self.submitted = tableValues[11]
        elif json is not None:
            for prop in self.allProps():
                if prop in json:
                    setattr(self, prop, json[prop])

    @property
    def id(self):
        """
        Table id. Internal to the dao, not exposed by the REST API
        """
        return self._id

    @id.setter
    def id(self, id):
        self._id = id

    @property
    def image_id(self):
        """
        Unique image_id, publicly exposed by the API and used to access information on the 
        image in various states (raw, cropped, and classified)
        """
        return self._image_id

    @image_id.setter
    def image_id(self, image_id):
        self._image_id = image_id

    @property
    def type(self):
        """
        Type of classification. AUVSI currently specifies three possible types:
        'standard', 'off_axis' or 'emergent'. Type must equal one of these to be 
        successfully inserted or modified in the table
        """
        return self._type

    @type.setter
    def type(self, type):
        self._type = type

    @property
    def latitude(self):
        """
        Geolocation latitude of the object 
        """
        return self._latitude

    @latitude.setter
    def latitude(self, latitude):
        self._latitude = latitude

    @property
    def longitude(self):
        """
        Geolocation longitude of the object
        """
        return self._longitude

    @longitude.setter
    def longitude(self, longitude):
        self._longitude = longitude

    @property
    def orientation(self):
        """
        Orientation of the character/object. AUVSI currently specifies 8 possible orientations:
        'N', 'NE', 'E', 'SE', 'S', 'SW', 'W' or 'NW'. Orientation must equal one of these to be
        successfully inserted or modified in the table.
        """
        return self._orientation

    @orientation.setter
    def orientation(self, orientation):
        self._orientation = orientation

    @property
    def shape(self):
        """
        Shape of the object for standar/off-axis types. AUVSI currently specifies 13 possible shapes:
        'circle', 'semicircle', 'quarter_circle', 'triangle', 'square', 'rectangle', 'trapezoid', 'pentagon', 'hexagon', 'heptagon', 'octagon', 'star' or 'cross'.
        Shape must equal one of these to be successfully inserted or modified in the table.
        """
        return self._shape

    @shape.setter
    def shape(self, shape):
        self._shape = shape

    @property
    def background_color(self):
        """
        Background color of the object for standard/off-axis types. AUVSI currently specifies 10 possible colors:
        'white', 'black', 'gray', 'red', 'blue', 'green', 'yellow', 'purple', 'brown' or 'orange'.
        Background_color must equal one of these to be successfully inserted or modified in the table
        """
        return self._background_color

    @background_color.setter
    def background_color(self, background_color):
        self._background_color = background_color

    @property
    def alphanumeric(self):
        """
        Alphanumeric within the target for standard/off-axis target types. At present AUVSI specifies that
        any uppercase alpha character or number may be within a target. Through in practice they have historicall
        only done alpha characters. Checking that this property is given/contains valid values is left to the user.
        """
        return self._alphanumeric

    @alphanumeric.setter
    def alphanumeric(self, alphanumeric):
        self._alphanumeric = alphanumeric

    @property
    def alphanumeric_color(self):
        """
        Color of the alphanumeric for a standard/off-axis type. Color specs are the same as background_color.
        Alphanumeric_color must be equal to one of the specified colors to be successfully inserted or modified in the table.
        """
        return self._alphanumeric_color

    @alphanumeric_color.setter
    def alphanumeric_color(self, alphanumeric_color):
        self._alphanumeric_color = alphanumeric_color

    @property
    def description(self):
        """
        Description of the emergent object.
        """
        return self._description

    @description.setter
    def description(self, description):
        self._description = description

    @property
    def submitted(self):
        """
        Boolean to indicate whether the classification has been submitted to the judges yet
        """
        return self._Submitted

    @submitted.setter
    def submitted(self, Submitted):
        self._Submitted = Submitted

    # TODO: this is hacky and i hate it
    def allProps(self):
        return ['id', 'image_id', 'type', 'latitude', 'longitude', 'orientation', 'shape', 'background_color', 'alphanumeric', 'alphanumeric_color', 'description', 'submitted']

    def toDict(self, exclude=None):
        """
        Return attributes contained in this model as a dictionary

        @type exclude: (string)
        @param exclude: Attribute names to exclude from the generated result

        @rtype: {string}
        @return: String dictionary of classification properties
        """
        dict = {}
        for attr, value in self.__dict__.items():
            corrected_name = attr[1:] # remove first underscore

            if exclude is None or corrected_name not in exclude:
                dict[corrected_name] = value.__str__()
        return dict