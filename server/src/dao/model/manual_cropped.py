from dao.model.point import point

class manual_cropped:

    def __init__(self, tableValues=None, json=None):
        if tableValues is not None:
            self.id = tableValues[0]
            self.raw_id = tableValues[1]
            self.time_stamp = tableValues[2]
            self.cropped_path = tableValues[3]
            self.crop_coordinate_tl = point(ptStr=tableValues[4])
            self.crop_coordinate_br = point(ptStr=tableValues[5])
            self.tapped = tableValues[6]
        elif json is not None:
            for prop in self.allProps():
                if prop in json and 'coordinate' not in prop:
                    setattr(self, prop, json[prop])
            # special handle coordinates
            if 'crop_coordinate_tl' in json:
                self.crop_coordinate_tl = point(ptStr=json['crop_coordinate_tl'])
            elif 'crop_coordinate_tl.x' in json:
                self.crop_coordinate_tl = point(x=json['crop_coordinate_tl.x'], y=json['crop_coordinate_tl.y'])

            if 'crop_coordinate_br' in json:
                self.crop_coordinate_br = point(json['crop_coordinate_br'])
            elif 'crop_coordinate_br.x' in json:
                self.crop_coordinate_br = point(x=json['crop_coordinate_br.x'], y=json['crop_coordinate_br.y'])
        else:
            # defaults:
            self.id = -1
            self.raw_id = -1
            self.tapped = False

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

    def allProps(self):
        return ['id', 'raw_id', 'time_stamp', 'cropped_path', 'crop_coordinate_tl', 'crop_coordinate_br', 'tapped']

    def toDict(self):
        dict = {}
        for attr, value in self.__dict__.items():
            corrected_name = attr[1:] # remove first underscore
            dict[corrected_name] = value.__str__()
        return dict

    def toJsonResponse(self):
        dict = self.toDict()
        
        # add crop point values independently:
        if hasattr(self, '_crop_coordinate_tl'):
            cropTl = self.crop_coordinate_tl.toDict()
            if cropTl is not None:
                dict['crop_coordinate_tl.x'] = cropTl['x']
                dict['crop_coordinate_tl.y'] = cropTl['y']
        if hasattr(self, '_crop_coordinate_br'):
            cropBr = self.crop_coordinate_br.toDict()
            if cropBr is not None:
                dict['crop_coordinate_br.x'] = cropBr['x']
                dict['crop_coordinate_br.y'] = cropBr['y']

        return dict

    def insertValues(self):
        return [self.raw_id, self.time_stamp, self.cropped_path, self.tapped]