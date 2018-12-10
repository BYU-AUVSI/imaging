import psycopg2
from dao.base_dao import BaseDAO
from dao.model.manual_cropped import manual_cropped

class ManualCroppedDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(ManualCroppedDAO, self).__init__(configFilePath)

    def upsertCropped(self, manualCropped):
        """
        Upserts a cropped image record. Will only insert or update
        values persented in the parameter

        @type manualCropped: manual_cropped
        @param manualCropped: manual_cropped object to update or insert. At a minimum must contain an image_id

        @rtype: int
        @return: Internal table id of the record inserted/updated if successful. Otherwise -1
        """

        insertImg = "INSERT INTO manual_cropped "
        updateCls = "UPDATE SET "

        insertValues = []
        insertClmnNames = '('
        insertClmnValues = ' VALUES('
        for clmn, value in manualCropped.toDict(exclude=('id',)).items():
            insertClmnNames += clmn + ', '
            if clmn != 'time_stamp':
                insertClmnValues += '%s, '
                updateCls += clmn + '= %s, '
            else:
                insertClmnValues += 'to_timestamp(%s), '
                updateCls += clmn + '= to_timestamp(%s), '
            insertValues.append(value.__str__())

        if not insertValues:
            return -1
        else:
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') ON CONFLICT (image_id) DO '
            updateCls = updateCls[:-2] + 'RETURNING id;'

        insertImg += insertClmnNames + insertClmnValues + updateCls

        return super(ManualCroppedDAO, self).getResultingId(insertImg, insertValues + insertValues)

    def addImage(self, manualCropped):
        """
        Adds the manually cropped image to the manual_cropped table

        @type manualCropped: manual_cropped
        @param manualCropped: manual_cropped image to insert. Should have image_id, time_stamp, cropped_path, and tapped

        @rtype: int
        @return: Internal table id of the manual_cropped entry if successfully inserted, otherwise -1
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

        @type id: int
        @param: The internal table id of the cropped image to retrieve.

        @rtype: manual_cropped
        @return: manual_cropped instance that was retrieved. If no image with that id exists, None
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
        """
        Get the next un-tapped cropped image for classification. This will retrieve the
        oldest cropped image where 'tapped'=FALSE.

        @rtype: manual_cropped
        @return: The next available manual_cropped image if one is available, otherwise None 
        """
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

    def getAll(self):
        """
        Get all the cropped image currently in the table

        @rtype: [outgoing_manual]
        @return: List of all cropped images in the manual_cropped table. If the table is empty, an empty list
        """
        selectAllSql = """SELECT id, image_id, date_part('epoch', time_stamp), cropped_path, crop_coordinate_tl, crop_coordinate_br, tapped
            FROM manual_cropped
            ORDER BY id;"""
        
        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        results = []
        for row in cur:
            manualCroppedRow = manual_cropped(row)
            results.append(manualCroppedRow)

        return results

    def updateImageByUID(self, id, updateContent):
        """
        Update the image with the specified image_id.

        @type id: int
        @param id: Image_id of the cropped information to update

        @type updateContent: {object}
        @param updateContent: Dictionary/JSON of attributes to update

        @rtype: manual_cropped
        @return: manual_cropped instance showing the current state of the now-updated row in the table. If the update fails, None
        """
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
        # this result id is a manual_cropped.id not image_id
        resultId = super(ManualCroppedDAO, self).getResultingId(updateStr, values)
        if resultId != -1:
            return self.getImage(resultId)
        else:
            return None
