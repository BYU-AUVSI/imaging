from dao.model.point import point

class manual_cropped:

    def __init__(self, tableValues=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.raw_id = tableValues[1]
            self.time_stamp = tableValues[2]
            self.cropped_path = tableValues[3]
            self.crop_coordinate_tl = point(ptStr=tableValues[4])
            self.crop_coordinate_br = point(ptStr=tableValues[5])
            self.tapped = tableValues[6]

    @property
    def id(self):    
        return self._id
    
    @id.setter
    def id(self, id):
        self._id = id

    @property
    def raw_id(self):
        return self._raw_id

    @raw_id.setter
    def raw_id(self, raw_id):
        self._raw_id = raw_id
    
    @property
    def time_stamp(self):    
        return self._time_stamp

    @time_stamp.setter
    def time_stamp(self, time_stamp):
        self._time_stamp = time_stamp

    @property
    def cropped_path(self):
        return self._cropped_path
    
    @cropped_path.setter
    def cropped_path(self, cropped_path):
        self._cropped_path = cropped_path

    @property
    def crop_coordinate_tl(self):
        return self._crop_coordinate_tl
    
    @crop_coordinate_tl.setter
    def crop_coordinate_tl(self, crop_coordinate_tl):
        self._crop_coordinate_tl = crop_coordinate_tl

    @property
    def crop_coordinate_br(self):
        return self._crop_coordinate_br
    
    @crop_coordinate_br.setter
    def crop_coordinate_br(self, crop_coordinate_br):
        self._crop_coordinate_br = crop_coordinate_br

    @property
    def tapped(self):
        return self._tapped
    
    @tapped.setter
    def tapped(self, tapped):
        self._tapped = tapped