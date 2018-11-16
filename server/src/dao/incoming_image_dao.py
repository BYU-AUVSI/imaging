from base_dao import BaseDAO
from model.incoming_image import incoming_image

class incoming_image_dao(BaseDAO):

    def __init__(self):
        pass
    
    def addImage(self, incomingImage):
        """
        Adds the specified image to the incoming_image table
        @type incomingImage: incoming_image
        @param incomingImage: The image to add to the database

        @rtype: boolean
        @return: True if successfully inserters, otherwise false
        """
        return True

