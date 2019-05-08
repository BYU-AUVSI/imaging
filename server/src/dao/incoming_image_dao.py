import psycopg2
from dao.base_dao import BaseDAO
from dao.model.incoming_image import incoming_image

class IncomingImageDAO(BaseDAO):
    """
    Handles interaction with raw images captured by the plane. Ros_ingest interacts
    with this DAO directly. On the REST side, most of its functionality is accessed through
    the /image/raw endpoint and the raw_image_handler module
    """

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
        if incomingImage is None:
            return -1

        insertStmt = """INSERT INTO incoming_image 
            (time_stamp, focal_length, image_path, manual_tap, autonomous_tap) 
            VALUES(to_timestamp(%s) AT TIME ZONE 'UTC', %s, %s, %s, %s) 
            RETURNING image_id;"""
        return super(IncomingImageDAO, self).getResultingId(insertStmt, incomingImage.insertValues())

    def getImage(self, id):
        """
        Attempts to get the image with the specified id

        @type id: int
        @param id: The id of the image to try and retrieve

        @rtype: incoming_image
        @return: An incoming_image with the info for that image if successfully found, otherwise None
        """
        selectImgById = """SELECT image_id, date_part('epoch', time_stamp), focal_length, image_path, manual_tap, autonomous_tap 
            FROM incoming_image 
            WHERE image_id = %s 
            LIMIT 1;"""
        selectedImage = super(IncomingImageDAO, self).basicTopSelect(selectImgById, (id,))
        
        if selectedImage is None:
            return None
        return incoming_image(selectedImage)

    def getNextImage(self, manual):
        """
        Attempts to get the next raw image not handled by the specified mode (manual or autonomous)

        @type manual: boolean
        @param manual: Whether to try and get the next image in manual's queue (True) or the autonomous queue (False)

        @rtype: incoming_image
        @return: An incoming_image with the info for that image if successfully found, otherwise None
        """

        # step 1: claim an image if possible:
        updateStmt = None
        if manual:
            updateStmt = """UPDATE incoming_image 
                SET manual_tap = TRUE 
                WHERE image_id = (
                    SELECT image_id 
                    FROM incoming_image 
                    WHERE manual_tap = FALSE 
                    ORDER BY image_id LIMIT 1
                ) RETURNING image_id;"""
        else:
            updateStmt = """UPDATE incoming_image 
                SET autonomous_tap = TRUE 
                WHERE image_id = (
                    SELECT image_id 
                    FROM incoming_image 
                    WHERE autonomous_tap = FALSE 
                    ORDER BY image_id LIMIT 1
                ) RETURNING image_id;"""

        cur = self.conn.cursor()
        cur.execute(updateStmt)

        row = cur.fetchone()
        if row is not None:
            # step 2: get the claimed image
            claimedId = row[0]
            cur.close()
            return self.getImage(claimedId)

        cur.close()
        # return none if we failed to claim an image
        return None

    def getAll(self):
        """
        Get info on all images currently in the table. Currently just used
        for testing
        """
        selectAllSql = """SELECT image_id, date_part('epoch', time_stamp), focal_length, image_path, manual_tap, autonomous_tap 
            FROM incoming_image;"""

        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        results = []
        for row in cur:
            incomingImg = incoming_image(row)
            results.append(incomingImg)

        return results