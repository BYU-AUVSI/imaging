import psycopg2
from dao.base_dao import BaseDAO
from dao.model.manual_cropped import manual_cropped

class ManualCroppedDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(ManualCroppedDAO, self).__init__(configFilePath)

    def getImage(self, id):
        """
        Attempts to get the image with the specified id

        @type id: int
        @param id: The id of the image to try and retrieve

        @rtype: manual_cropped
        @return: A manual_cropped image with the info for that image if successfully found, otherwise None
        """

        selectImgById = """SELECT id, raw_id, date_part('epoch', time_stamp), cropped_path, crop_coordinate_tl, crop_coordinate_br, tapped
            FROM manual_cropped
            WHERE id = %s
            LIMIT 1;"""
        selectedImage = super(ManualCroppedDAO, self).basicTopSelect(selectImgById, (id,))

        if selectedImage is None:
            return None
            
        return manual_cropped(selectedImage)