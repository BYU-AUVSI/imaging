import psycopg2
from dao.base_dao import BaseDAO
from dao.model.manual_cropped import manual_cropped

class ManualCroppedDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(ManualCroppedDAO, self).__init__(configFilePath)

    def addImage(self, manualCropped):
        """
        Adds the manually cropped image to the manual_cropped table
        """
        
        insertImg = """INSERT INTO manual_cropped
            (raw_id, time_stamp, cropped_path, tapped) 
            VALUES(%s, to_timestamp(%s), %s, %s) 
            RETURNING id;"""

        return super(ManualCroppedDAO, self).basicInsert(insertImg, manualCropped.insertValues())
    

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

    def getNextImage(self):
        # step 1: claim an image if possible
        updateStmt = """UPDATE manual_cropped 
            SET tapped = TRUE 
            WHERE id = (
                SELECT id 
                FROM manual_cropped 
                WHERE tapped = FALSE 
                ORDER BY id LIMIT 1
            ) RETURNING id;"""

        cur = self.conn.cursor()
        cur.execute(updateStmt)

        row = cur.fetchone()
        if row is not None:
            claimedId = row[0]
            cur.close()
            return self.getImage(claimedId)

        # failed to reserve an image (none available)
        cur.close()
        return None