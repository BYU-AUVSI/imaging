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
    EMERGENT_KEYS_TO_INCLUDE = ['type', 'latitude', 'longitude', 'description', 'autonomous']

    HELPER_VALUES = ['target', 'submitted', 'crop_path']

    def __init__(self, sqlRow=None, outgoingManualOrAutonomous=None, autonomous_in=None):
        
        # initialize all the attributes to None
        self.target = None
        """Target id from the corresponding (autonomous or manual) outgoing table"""
        self.autonomous = None
        """Whether this target was generated by the autonomous classifier (True) or the manual (False)"""
        self.type = None
        """Type of classification. AUVSI currently specifies three possible types:
        'standard', 'off_axis' or 'emergent'. Type must equal one of these to be 
        successfully inserted or modified in the table"""
        self.crop_path = None
        """Path to where the image is saved on the server filesystem"""
        self.latitude = None
        """Geolocation latitude of the object """
        self.longitude = None
        """Geolocation longitude of the object"""
        self.orientation = None
        """See outgoing_classification model docs"""
        self.shape = None
        """See outgoing_classification model docs"""
        self.background_color = None
        """See outgoing_classification model docs"""
        self.alphanumeric = None
        """See outgoing_classification model docs"""
        self.alphanumeric_color = None
        """See outgoing_classification model docs"""
        self.description = None
        """See outgoing_classification model docs"""
        self.submitted = None
        """Submission status of this target to the judge server via interop. 
        NOTE: this submitted column is different from the outgoing_classification 
        submitted column. Outgoing_manual(/autonomous) submitted column is an 
        indication as to whether the classification for a target has been submitted 
        to this table. Submitted here indicates whether this final target has been 
        submitted to the judge interop server"""

        if sqlRow is not None:
            for i in range(0, len(self.allProps())):
                setattr(self, self.allProps()[i], sqlRow[i])

        if autonomous_in is not None:
            self.autonomous = autonomous_in

        if outgoingManualOrAutonomous is not None:
            outDict = outgoingManualOrAutonomous.toDict()
            for key in self.allProps():
                if key in outDict:
                    setattr(self, key, outDict[key])


    # TODO: this is hacky and i hate it
    def allProps(self):
        return ['target', 'autonomous', 'type', 'crop_path', 'latitude', 'longitude', 'orientation', 'shape', 'background_color', 'alphanumeric', 'alphanumeric_color', 'description', 'submitted']

    def toDict(self, exclude=None):
        dict = {}
        for attr, value in self.__dict__.items():
            dict[attr] = value
        return dict

    def toAuvsiJson(self, exclude=None):
        dict = {}
        if hasattr(self, 'type') and self.type == 'emergent':
            # generate an emergent target output
            for attr, value in self.__dict__.items():
                if isinstance(value, str):
                    value = value.upper()
                if attr in self.EMERGENT_KEYS_TO_INCLUDE:
                    dict[attr] = value
                elif attr not in self.HELPER_VALUES:
                    dict[attr] = None
        else:
            for attr, value in self.__dict__.items():
                if isinstance(value, str):
                        value = value.upper()
                if attr not in self.KEYS_TO_EXCLUDE:
                    dict[attr] = value
                elif attr not in self.HELPER_VALUES:
                    dict[attr] = None

        if exclude is not None:
            for keyToExclude in exclude:
                if keyToExclude in dict:
                    del dict[keyToExclude]
    
        return dict
