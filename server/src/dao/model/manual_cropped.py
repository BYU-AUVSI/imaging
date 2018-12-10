from dao.model.point import point

class manual_cropped:
    """
    Model class for the manual_cropped table. Properties and helper
    methods for images cropped by the manual client
    """

    def __init__(self, tableValues=None, json=None):
        """
        Accepts various formats to instantiate this model object

        @type tableValues: [object]
        @param tableValues: List of table values, in table column order

        @type json: {object}
        @param json: Json dictionary of table values. Used by the REST API when receiving data
        """
        if tableValues is not None:
            self.id = tableValues[0]
            self.image_id = tableValues[1]
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
            self.image_id = -1
            self.tapped = False

    @property
    def id(self):
        """
        Table id. Internal to the dao, not exposed by the REST API
        """
        return self._id
    
    @id.setter
    def id(self, id):
        self._id = id

    @property
    def image_id(self):
        """
        Unique image_id, publicly exposed by the API and used to access information on the 
        image in various states (raw, cropped, and classified)
        """
        return self._image_id

    @image_id.setter
    def image_id(self, image_id):
        self._image_id = image_id
    
    @property
    def time_stamp(self):
        """
        UTC Unix epoch timestamp as float. Indicates when the cropped image was inserted.
        """
        return self._time_stamp

    @time_stamp.setter
    def time_stamp(self, time_stamp):
        self._time_stamp = time_stamp

    @property
    def cropped_path(self):
        """
        Path to where the image is saved on the server filesystem
        """
        return self._cropped_path
    
    @cropped_path.setter
    def cropped_path(self, cropped_path):
        self._cropped_path = cropped_path

    @property
    def crop_coordinate_tl(self):
        """
        Point object specifying the top-left (tl) coordinate of where the raw image was cropped to produce this cropped image
        """
        return self._crop_coordinate_tl
    
    @crop_coordinate_tl.setter
    def crop_coordinate_tl(self, crop_coordinate_tl):
        self._crop_coordinate_tl = crop_coordinate_tl

    @property
    def crop_coordinate_br(self):
        """
        Point object specifying the bottom-right (br) coordinate of where the raw image was cropped to produce this cropped image
        """
        return self._crop_coordinate_br
    
    @crop_coordinate_br.setter
    def crop_coordinate_br(self, crop_coordinate_br):
        self._crop_coordinate_br = crop_coordinate_br

    @property
    def tapped(self):
        """
        Boolean to specify whether the cropped image has been tapped by the classifier
        """
        return self._tapped
    
    @tapped.setter
    def tapped(self, tapped):
        self._tapped = tapped

    # TODO: this is hacky and i hate it
    def allProps(self):
        return ['id', 'image_id', 'time_stamp', 'cropped_path', 'crop_coordinate_tl', 'crop_coordinate_br', 'tapped']

    def toDict(self, exclude=None):
        """
        Return attributes contained in this model as a dictionary

        @type exclude: (string)
        @param exclude: Attribute names to exclude from the generated result

        @rtype: {string}
        @return: String dictionary of cropped image properties
        """
        dict = {}
        for attr, value in self.__dict__.items():
            corrected_name = attr[1:] # remove first underscore
            # add everything we weren't explicitly told to exclude
            if exclude is None or corrected_name not in exclude:
                dict[corrected_name] = value.__str__()
        return dict

    def toJsonResponse(self, exclude=None):
        """
        Produce a dictionary of this cropped instance. This is very similar to the 
        toDict method, but adds a few values to the json to separate crop coordinates
        into x and y

        @type exclude: (string)
        @param exclude: Attribute names to exclude from the generated result

        @rtype: {object}
        @return: Dictionary of attributes stored in this instance, not including those attributes specified in exclude.
        """
        dict = self.toDict(exclude)
        
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
        """
        Get the cropped image as an object list.
        The properties are ordered as they would be for a
        barebones table insert. (In many cases crop coordinates are provided for the
        initial insert, so this method isn't used)

        @rtype: [object]
        @return: Ordered object list - image_id, time_stamp, cropped_path, tapped
        """
        return [self.image_id, self.time_stamp, self.cropped_path, self.tapped]