class incoming_state:

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
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, roll):
        self._roll = roll

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, pitch):
        self._pitch = pitch

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, yaw):
        self._yaw = yaw
