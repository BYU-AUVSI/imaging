class incoming_image:

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