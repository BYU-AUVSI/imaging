from base_dao import BaseDAO
from model.incoming_image import incoming_image

class IncomingImageDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(IncomingImageDAO, self).__init__(configFilePath)
    
    def addImage(self, incomingImage):
        """
        Adds the specified image to the incoming_image table
        @type incomingImage: incoming_image
        @param incomingImage: The image to add to the database

        @rtype: int
        @return: Id of image if successfully inserted, otherwise -1
        """
        insertStmt = "INSERT INTO incoming_image (time_stamp, image_path, manual_tap, autonomous_tap) VALUES(to_timestamp(%s), %s, %s, %s) RETURNING id;"
        return super(IncomingImageDAO, self).basicInsert(insertStmt, incomingImage.insertValues())

