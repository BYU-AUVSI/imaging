class incoming_image:
    """
    Model class for the Raw Image table. Properties and helper
    methods for raw images from the camera.
    """

    def __init__(self, tableValues=None):
        self.focal_length = None # so errors arent thrown if an incoming image doesnt have the data for some reason (ie: playing back an old bag)
        if tableValues is not None:
            self.image_id = tableValues[0]
            self.time_stamp = tableValues[1]
            self.focal_length = tableValues[2]
            self.image_path = tableValues[3]
            self.manual_tap = tableValues[4]
            self.autonomous_tap = tableValues[5]

    @property
    def image_id(self):
        """
        Table id for this image. This image_id is used throughout
        The other tables as a unique identifier back to various states
        of the image.
        """
        return self._id

    @image_id.setter
    def image_id(self, id):
        self._id = id

    @property
    def time_stamp(self):
        """
        UTC Unix epoch timestamp as float.
        """
        return self._time_stamp

    @time_stamp.setter
    def time_stamp(self, time_stamp):
        self._time_stamp = time_stamp

    @property
    def focal_length(self):
        """
        The focal length of the image at the time of capture. In mm as float.
        """
        return self._focal_length

    @focal_length.setter
    def focal_length(self, focal_length):
        self._focal_length = focal_length

    @property
    def image_path(self):
        """
        Path to where the image is saved on the server filesystem
        """
        return self._image_path

    @image_path.setter
    def image_path(self, image_path):
        self._image_path = image_path

    @property
    def manual_tap(self):
        """
        Boolean indicating whether this image has been 'tapped'
        (aka seen) by the manual imaging client
        """
        return self._manual_tap

    @manual_tap.setter
    def manual_tap(self, manual_tap):
        self._manual_tap = manual_tap

    @property
    def autonomous_tap(self):
        """
        Boolean indicating whether this image has been 'tapped'
        (aka seen) by the autonomous imaging client
        """
        return self._autonomous_tap

    @autonomous_tap.setter
    def autonomous_tap(self, autonomous_tap):
        self._autonomous_tap = autonomous_tap

    def insertValues(self):
        """
        Get the raw image as an object list.
        The properties are ordered as they would be for a
        normal table insert

        @rtype: [object]
        @return: Ordered object list - time_stamp, image_path, manual_tap, autonomous_tap
        """
        return [self.time_stamp, self.focal_length, self.image_path, self.manual_tap, self.autonomous_tap]

    def toDict(self):
        """
        Return properties contained in this model as a dictionary

        @rtype: {object}
        @return: Object dictionary of raw image properties
        """
        dict = {}
        for attr, value in self.__dict__.items():
            corrected_name = attr[1:] # remove first underscore
            dict[corrected_name] = value.__str__()
        return dict

    def __str__(self):
        """
        Debug convenience method to get this instance as a string
        """
        return """IncomingImage:\n
            \tid: {}\n
            \ttime_stamp: {}\n
            \tfocal_length: {}\n
            \timage_path: {}\n
            \tmanual_tap: {}\n
            \tautonomous_tap: {}""".format(self.image_id, self.time_stamp, self.focal_length, self.image_path, self.manual_tap, self.autonomous_tap)