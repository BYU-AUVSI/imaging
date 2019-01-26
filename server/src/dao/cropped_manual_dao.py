import psycopg2
from dao.cropped_image_dao import CroppedImageDAO
from dao.model.cropped_manual import cropped_manual

class CroppedManualDAO(CroppedImageDAO):

    def __init__(self, configFilePath):
        super(CroppedManualDAO, self).__init__(configFilePath, 'cropped_manual', newModelFromRow)


def newModelFromRow(row, json=None):
    return cropped_manual(row, json=json)