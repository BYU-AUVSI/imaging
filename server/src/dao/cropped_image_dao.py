import psycopg2
from dao.base_dao import BaseDAO
from dao.model.manual_cropped import manual_image

class CroppedImageDAO(BaseDAO):

    def __init__(self, configFilePath, croppedTableName, modelGenerator):
        """
        Initialize the cropped image dao, which holds common functionality for 
        interaction with both the cropped_manual and cropped_autonomous table

        @param modelGenerator: the class that can generate the proper model to 
            represent a row from the resulting query. Generally this is a reference to 
            some function in the child class (ie: cropped_manual_dao holds a function
            that can be called to create a new cropped_manual object while cropped_autonomous_dao
            holds a similar function that can create a cropped_autonomous object
        """
        super(CroppedImageDAO, self).__init__(configFilePath)
        self.croppedTableName = croppedTableName
        self.modelGenerator = modelGenerator

    def upsertCropped(self, manualCropped):
        """
        Upserts a cropped image record. Will only insert or update
        values persented in the parameter

        @type manualCropped: manual_cropped
        @param manualCropped: manual_cropped object to update or insert. At a minimum must contain an image_id

        @rtype: int
        @return: Internal table id of the record inserted/updated if successful. Otherwise -1
        """
        if manualCropped is None:
            return -1

        insertImg = "INSERT INTO " + self.croppedTableName
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
                insertClmnValues += "to_timestamp(%s) AT TIME ZONE 'UTC', "
                updateCls += clmn + "= to_timestamp(%s) AT TIME ZONE 'UTC', "
            insertValues.append(value.__str__())

        if manualCropped.id is not None and manualCropped.id != -1:
            insertClmnNames  += 'id, '
            insertClmnValues += '%s, '
            updateCls += 'id= %s, '
            insertValues.append(manualCropped.id)
        if not insertValues:
            return -1
        else:
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') ON CONFLICT (id) DO '
            updateCls = updateCls[:-2] + 'RETURNING id;'

        insertImg += insertClmnNames + insertClmnValues + updateCls

        return super(CroppedImageDAO, self).getResultingId(insertImg, insertValues + insertValues)

    def addImage(self, manualCropped):
        """
        Adds the manually cropped image to the manual_cropped table

        @type manualCropped: manual_cropped
        @param manualCropped: manual_cropped image to insert. Should have image_id, time_stamp, cropped_path, and tapped

        @rtype: int
        @return: Internal table id of the manual_cropped entry if successfully inserted, otherwise -1
        """
        if manualCropped is None:
            return -1
        
        insertImg = """INSERT INTO """ + self.croppedTableName + """
            (image_id, time_stamp, cropped_path) 
            VALUES(%s, to_timestamp(%s) AT TIME ZONE 'UTC', %s) 
            RETURNING id;"""

        return super(CroppedImageDAO, self).getResultingId(insertImg, manualCropped.insertValues())

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
            FROM """ + self.croppedTableName + """
            WHERE id = %s
            LIMIT 1;"""
        selectedImage = super(CroppedImageDAO, self).basicTopSelect(selectImgById, (id,))

        if selectedImage is None:
            return None
        return self.modelGenerator(selectedImage)

    def getNextImage(self):
        """
        Get the next un-tapped cropped image for classification. This will retrieve the
        oldest cropped image where 'tapped'=FALSE.

        @rtype: manual_cropped
        @return: The next available manual_cropped image if one is available, otherwise None 
        """
        # step 1: claim an image if possible
        # this gets the oldest (aka lowest) id with tapped = False
        updateStmt = """UPDATE """ + self.croppedTableName + """ 
            SET tapped = TRUE 
            WHERE id = (
                SELECT id 
                FROM """ + self.croppedTableName + """ 
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
        @return: List of all cropped images in a cropped image table. If the table is empty, an empty list
        """
        selectAllSql = """SELECT id, image_id, date_part('epoch', time_stamp), cropped_path, crop_coordinate_tl, crop_coordinate_br, tapped
            FROM """ + self.croppedTableName + """
            ORDER BY id;"""
        
        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        results = []
        for row in cur:
            manualCroppedRow = self.modelGenerator(row)
            results.append(manualCroppedRow)

        return results


    def updateImage(self, id, updateContent):
        """
        Update the image with the specified crop_id.

        @type id: int
        @param id: Crop_id of the cropped information to update

        @type updateContent: {object}
        @param updateContent: Dictionary/JSON of attributes to update

        @rtype: manual_cropped
        @return: manual_cropped instance showing the current state of the now-updated row in the table. If the update fails, None
        """
        img = self.modelGenerator(json=updateContent)
        updateStr = "UPDATE " + self.croppedTableName + " SET "

        values = []
        for clmn, value in img.toDict().items():
            if clmn == 'time_stamp':
                updateStr += clmn + "= to_timestamp(%s) AT TIME ZONE 'UTC'"
            else:
                updateStr += clmn + "= %s "

            values.append(value.__str__())
            updateStr += ", "
        
        updateStr = updateStr[:-2] # remove last space/comma
        updateStr += " WHERE id = %s RETURNING id;"
        values.append(id)
        
        resultId = super(CroppedImageDAO, self).getResultingId(updateStr, values)
        if resultId != -1:
            return self.getImage(resultId)
        else:
            return None
