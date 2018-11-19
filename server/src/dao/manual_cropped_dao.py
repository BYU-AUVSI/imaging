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
            (image_id, time_stamp, cropped_path, tapped) 
            VALUES(%s, to_timestamp(%s), %s, %s) 
            RETURNING id;"""

        return super(ManualCroppedDAO, self).getResultingId(insertImg, manualCropped.insertValues())
    
    def getImageByUID(self, id):
        """
        Attempts to get the image with the specified universal-identifier

        @type id: int
        @param id: The id of the image to try and retrieve

        @rtype: manual_cropped
        @return: A manual_cropped image with the info for that image if successfully found, otherwise None
        """
        selectImgById = """SELECT id, image_id, date_part('epoch', time_stamp), cropped_path, crop_coordinate_tl, crop_coordinate_br, tapped
            FROM manual_cropped
            WHERE image_id = %s
            LIMIT 1;"""
        selectedImage = super(ManualCroppedDAO, self).basicTopSelect(selectImgById, (id,))

        if selectedImage is None:
            return None
        return manual_cropped(selectedImage)

    def getImage(self, id):
        """
        Attempts to get the image with the specified manual_cropped table id.
        NOTE: the different between getImageByUID. getImageByUID selects on the image_id
        which is a universal id for an image shared across the incoming_image, manual_cropped and outgoing_manual tables
        """
        selectImgById = """SELECT id, image_id, date_part('epoch', time_stamp), cropped_path, crop_coordinate_tl, crop_coordinate_br, tapped
            FROM manual_cropped
            WHERE id = %s
            LIMIT 1;"""
        selectedImage = super(ManualCroppedDAO, self).basicTopSelect(selectImgById, (id,))

        if selectedImage is None:
            return None
        return manual_cropped(selectedImage)


    def getNextImage(self):
        # step 1: claim an image if possible
        # this gets the oldest (aka lowest) id with tapped = False
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

    def updateImageByUID(self, id, updateContent):
        # push json into the manual_cropped model. this will
        # extract any relevant attributes
        img = manual_cropped(json=updateContent)
        updateStr = "UPDATE manual_cropped SET "

        # compose the update string:
        values = []
        for clmn, value in img.toDict().items():
            updateStr += clmn + "= %s, "
            values.append(value.__str__())

        updateStr = updateStr[:-2] # remove last space/comma
        updateStr += " WHERE image_id = %s RETURNING id;"
        values.append(id)
        print(updateStr)
        # this result id is a manual_cropped.id not image_id
        resultId = super(ManualCroppedDAO, self).getResultingId(updateStr, values)
        print("==========>> {}".format(resultId))
        if resultId != -1:
            return self.getImage(resultId)
        else:
            return -1
