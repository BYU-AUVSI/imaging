class incoming_image:

    def __init__(self, tableValues=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.time_stamp = tableValues[1]
            self.image_path = tableValues[2]
            self.manual_tap = tableValues[3]
            self.autonomous_tap = tableValues[4]

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        self._id = id

    @property
    def time_stamp(self):
        return self._time_stamp

    @time_stamp.setter
    def time_stamp(self, time_stamp):
        self._time_stamp = time_stamp

    @property
    def image_path(self):
        return self._image_path

    @image_path.setter
    def image_path(self, image_path):
        self._image_path = image_path

    @property
    def manual_tap(self):
        return self._manual_tap

    @manual_tap.setter
    def manual_tap(self, manual_tap):
        self._manual_tap = manual_tap

    @property
    def autonomous_tap(self):
        return self._autonomous_tap

    @autonomous_tap.setter
    def autonomous_tap(self, autonomous_tap):
        self._autonomous_tap = autonomous_tap

    def insertValues(self):
        return [self.time_stamp, self.image_path, self.manual_tap, self.autonomous_tap]

    def __str__(self):
        return "IncomingImage:\n\tid: {}\n\ttime_Stamp: {}\n\timage_path: {}\n\tmanual_tap: {}\n\tautonomous_tap: {}".format(self.id, self.time_stamp, self.image_path, self.manual_tap, self.autonomous_tap)