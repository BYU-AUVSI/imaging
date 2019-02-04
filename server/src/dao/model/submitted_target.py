class submitted_target:
    """
    This model class helps represent the final form of an AUVSI target submission.
    Essentially just removes helper columns from outgoing_ classification models

    NOTE: for this to work properly (and to make maintenance and understanding easier)
    attribute names produced by toDict and toAuvsiJson should match the names
    of files within the InteropImage ROS message
    """

    # A list of keys to exclude in the output .json file generated by the DAO
    # for standard and off_axis target types
    KEYS_TO_EXCLUDE = ['target', 'crop_path', 'description', 'submitted']

    # a list of keys to include in the output .json file generated by the DAO
    # for emergent target type
    EMERGENT_KEYS_TO_INCLUDE = ['latitude', 'longitude', 'description', 'autonomous']

    def __init__(self, sqlRow=None, outgoingManualOrAutonomous=None, autonomous=None):
        
        self.target = None
        self.autonomous = None
        self.type = None
        self.crop_path = None
        self.latitude = None
        self.longitude = None
        self.orientation = None
        self.shape = None
        self.background_color = None
        self.alphanumeric = None
        self.alphanumeric_color = None
        self.description = None
        self.submitted = None

        if sqlRow is not None:
            for i in range(0, len(self.allProps())):
                setattr(self, self.allProps()[i], sqlRow[i])

        if autonomous is not None:
            self.autonomous = autonomous
        else:
            self.autonomous = False

        if outgoingManualOrAutonomous is not None:
            outDict = outgoingManualOrAutonomous.toDict()
            for key in self.allProps():
                if key in outDict:
                    setattr(self, key, outDict[key])

    
    
    # @property
    # def target(self):
    #     """
    #     Target Id this classification is being bundled into. Generally this is an internal
    #     only column managed by the DAO, but can be manually modified if needed. Classifications
    #     that are similar enough are placed into a 'target bin'. Since it is likely multiple images
    #     will be taken of each target, this prevents us from submitting multiple images of the same
    #     target to AUVSI.
    #     """
    #     return self._target

    # @target.setter
    # def target(self, target):
    #     self._target = target

    # @property
    # def autonomous(self):
    #     return self._autonomous

    # @autonomous.setter
    # def autonomous(self, autonomous):
    #     self._autonomous = autonomous

    # @property
    # def type(self):
    #     """
    #     Type of classification. AUVSI currently specifies three possible types:
    #     'standard', 'off_axis' or 'emergent'. Type must equal one of these to be 
    #     successfully inserted or modified in the table
    #     """
    #     return self._type

    # @type.setter
    # def type(self, type):
    #     self._type = type

    # @property
    # def crop_path(self):
    #     """
    #     crop_path references the absolute server path of the cropped image 
    #     to use for submission.
    #     """
    #     return self._crop_path

    # @crop_path.setter
    # def crop_path(self, crop_path):
    #     self._crop_path = crop_path

    # @property
    # def latitude(self):
    #     """
    #     Geolocation latitude of the object 
    #     """
    #     return self._latitude

    # @latitude.setter
    # def latitude(self, latitude):
    #     self._latitude = latitude

    # @property
    # def longitude(self):
    #     """
    #     Geolocation longitude of the object
    #     """
    #     return self._longitude

    # @longitude.setter
    # def longitude(self, longitude):
    #     self._longitude = longitude

    # @property
    # def orientation(self):
    #     """
    #     Orientation of the character/object. AUVSI currently specifies 8 possible orientations:
    #     'N', 'NE', 'E', 'SE', 'S', 'SW', 'W' or 'NW'. Orientation must equal one of these to be
    #     successfully inserted or modified in the table.
    #     """
    #     return self._orientation

    # @orientation.setter
    # def orientation(self, orientation):
    #     self._orientation = orientation

    # @property
    # def shape(self):
    #     """
    #     Shape of the object for standar/off-axis types. AUVSI currently specifies 13 possible shapes:
    #     'circle', 'semicircle', 'quarter_circle', 'triangle', 'square', 'rectangle', 'trapezoid', 'pentagon', 'hexagon', 'heptagon', 'octagon', 'star' or 'cross'.
    #     Shape must equal one of these to be successfully inserted or modified in the table.
    #     """
    #     return self._shape

    # @shape.setter
    # def shape(self, shape):
    #     self._shape = shape

    # @property
    # def background_color(self):
    #     """
    #     Background color of the object for standard/off-axis types. AUVSI currently specifies 10 possible colors:
    #     'white', 'black', 'gray', 'red', 'blue', 'green', 'yellow', 'purple', 'brown' or 'orange'.
    #     Background_color must equal one of these to be successfully inserted or modified in the table
    #     """
    #     return self._background_color

    # @background_color.setter
    # def background_color(self, background_color):
    #     self._background_color = background_color

    # @property
    # def alphanumeric(self):
    #     """
    #     Alphanumeric within the target for standard/off-axis target types. At present AUVSI specifies that
    #     any uppercase alpha character or number may be within a target. Through in practice they have historicall
    #     only done alpha characters. Checking that this column is given/contains valid values is left to the user.
    #     """
    #     return self._alphanumeric

    # @alphanumeric.setter
    # def alphanumeric(self, alphanumeric):
    #     self._alphanumeric = alphanumeric

    # @property
    # def alphanumeric_color(self):
    #     """
    #     Color of the alphanumeric for a standard/off-axis type. Color specs are the same as background_color.
    #     Alphanumeric_color must be equal to one of the specified colors to be successfully inserted or modified in the table.
    #     """
    #     return self._alphanumeric_color

    # @alphanumeric_color.setter
    # def alphanumeric_color(self, alphanumeric_color):
    #     self._alphanumeric_color = alphanumeric_color

    # @property
    # def description(self):
    #     """
    #     Description of the emergent object.
    #     """
    #     return self._description

    # @description.setter
    # def description(self, description):
    #     self._description = description

    # @property
    # def submitted(self):
    #     """
    #     Status of the target (either 'pending' submission or 'submitted')
    #     """
    #     return self._submitted

    # @submitted.setter
    # def submitted(self, submitted):
        # self._submitted = submitted

    # TODO: this is hacky and i hate it
    def allProps(self):
        return ['target', 'autonomous', 'type', 'crop_path', 'latitude', 'longitude', 'orientation', 'shape', 'background_color', 'alphanumeric', 'alphanumeric_color', 'description', 'submitted']

    def toDict(self, exclude=None):
        dict = {}
        for attr, value in self.__dict__.items():
            corrected_name = attr#[1:] # remove first underscore
            dict[corrected_name] = value
        return dict

    def toAuvsiJson(self, exclude=None):
        dict = {}
        if hasattr(self, 'type') and self.type == 'emergent':
            # generate an emergent target output
            for attr, value in self.__dict__.items():
                corrected_name = attr#[1:] # remove first underscore

                if corrected_name in self.EMERGENT_KEYS_TO_INCLUDE:
                    dict[corrected_name] = value
        else:
            for attr, value in self.__dict__.items():
                corrected_name = attr#[1:] # remove first underscore

                if corrected_name not in self.KEYS_TO_EXCLUDE:
                    dict[corrected_name] = value            
        
        if exclude is not None:
            for keyToExclude in exclude:
                if keyToExclude in dict:
                    del dict[keyToExclude]
    
        return dict