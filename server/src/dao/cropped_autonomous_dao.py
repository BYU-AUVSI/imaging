import psycopg2
from dao.cropped_image_dao import CroppedImageDAO
from dao.model.cropped_autonomous import cropped_autonomous

class CroppedAutonomousDAO(CroppedImageDAO):

    def __init__(self, configFilePath):
        super(CroppedAutonomousDAO, self).__init__(configFilePath, 'cropped_autonomous', newModelFromRow)


def newModelFromRow(row, json=None):
    """
    Create a new cropped_autonomous model object given a list of sql values from the table,
    or a json dictionary
    """
    return cropped_autonomous(row, json=json)