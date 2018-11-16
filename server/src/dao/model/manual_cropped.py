class ManualCropped:

    def getId(self):    
        return self.id
    
    def setId(self, id):
        self.id = id

    def getRawId(self):
        return self.raw_id

    def setRawId(self, raw_id):
        self.raw_id = raw_id
    
    def getTimeStamp(self):    
        return self.time_stamp

    def setTimeStamp(self, time_stamp):
        self.time_stamp = time_stamp

    def getCroppedPath(self):
        return self.cropped_path
    
    def setCroppedPath(self, cropped_path):
        self.cropped_path = cropped_path

    def getCropCoordinateTl(self):
        return self.crop_coordinate_tl
    
    def setCropCoordinateTl(self, crop_coordinate_tl):
        self.crop_coordinate_tl = crop_coordinate_tl

    def getCropCoordinateBr(self):
        return self.crop_coordinate_br
    
    def setCropCoordinateBr(self, crop_coordinate_br):
        self.crop_coordinate_br = crop_coordinate_br

    def getTapped(self):
        return self.tapped
    
    def setTapped(self, tapped):
        self.tapped = tapped

    id = property(getId, setId)
    raw_id = property(getRawId, setRawId)
    time_stamp = property(getTimeStamp, setTimeStamp)
    cropped_path = property(getCroppedPath, setCroppedPath)
    crop_coordinate_tl = property(getCropCoordinateTl, setCropCoordinateTl)
    crop_coordinate_br = property(getCropCoordinateBr, setCropCoordinateBr)
    tapped = property(getTapped, setTapped)