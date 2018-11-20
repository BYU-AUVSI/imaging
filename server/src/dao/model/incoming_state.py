class incoming_state:

    def __init__(self, tableValues=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.time_stamp = tableValues[1]
            self.roll = tableValues[2]
            self.pitch = tableValues[3]
            self.yaw = tableValues[4]

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

    def insertValues(self):
        return [self.time_stamp, self.roll, self.pitch, self.yaw]

    def toDict(self):
        dict = {}
        dict['id'] = self.id
        dict['time_stamp'] = self.time_stamp
        dict['roll'] = self.roll
        dict['pitch'] = self.pitch
        dict['yaw'] = self.yaw
        return dict

    def __str__(self):
        return """IncomingState:\n
            \tid: {}\n
            \ttime_stamp: {}\n
            \troll: {}\n
            \tpitch: {}\n
            \tyaw: {}""".format(self.id, self.time_stamp, self.roll, self.pitch, self.yaw)
