class incoming_state:
    """
    Model class for the State table. Properties and helper methods
    for state measurements.
    """

    def __init__(self, tableValues=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.time_stamp = tableValues[1]
            self.roll = tableValues[2]
            self.pitch = tableValues[3]
            self.yaw = tableValues[4]

    @property
    def id(self):
        """
        Table id for this measurement. Empty when inserting a new measurement.
        """
        return self._id

    @id.setter
    def id(self, id):
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
    def roll(self):
        """
        Measurement roll as a float
        """
        return self._roll

    @roll.setter
    def roll(self, roll):
        self._roll = roll

    @property
    def pitch(self):
        """
        Measurement pitch as a float
        """
        return self._pitch

    @pitch.setter
    def pitch(self, pitch):
        self._pitch = pitch

    @property
    def yaw(self):
        """
        Measurement yaw as a float
        """
        return self._yaw

    @yaw.setter
    def yaw(self, yaw):
        self._yaw = yaw

    def insertValues(self):
        """
        Get the gps measurement as an object list.
        The properties are ordered as they would be for a
        normal table insert

        @rtype: [object]
        @return: Ordered object list - time_stamp, roll, pitch, yaw
        """
        return [self.time_stamp, self.roll, self.pitch, self.yaw]

    def toDict(self):
        """
        Return properties contained in this measurement as a dictionary

        @rtype: {object}
        @return: Object dictionary of state measurement properties
        """
        dict = {}
        dict['id'] = self.id
        dict['time_stamp'] = self.time_stamp
        dict['roll'] = self.roll
        dict['pitch'] = self.pitch
        dict['yaw'] = self.yaw
        return dict

    def __str__(self):
        """
        Debug convenience method to get this instance as a string
        """
        return """IncomingState:\n
            \tid: {}\n
            \ttime_stamp: {}\n
            \troll: {}\n
            \tpitch: {}\n
            \tyaw: {}""".format(self.id, self.time_stamp, self.roll, self.pitch, self.yaw)
