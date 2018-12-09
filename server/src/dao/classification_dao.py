import psycopg2
from dao.base_dao import BaseDAO

class ClassificationDAO(BaseDAO):
    """
    Does most of the heavy lifting for classification tables: outgoing_autonomous and outgoing_manual.
    Contains general database methods which work for both types.
    """

    def __init__(self, configFilePath, outgoingTableName):
        super(ClassificationDAO, self).__init__(configFilePath)
        self.outgoingTableName = outgoingTableName # 0 for autonomous. 1 for manual

    def upsertClassification(self, classification):
        """
        Upserts a classification record.
        If the image_id given in the classification object already exists within the table, the corresponding record
        is updated. If it doesn't exist, then we insert a new record.

        @type classification: outgoing_manual
        @type classification: outgoing_autonomous
        @param classification: The outgoing_autonomous or manual classification to upsert. 
                                Note that these objects do not require all classification properties to
                                be successfully upserted. At a minimum it must have image_id.
                                ie: upsert could work if you provided a classification object with only
                                image_id, shape and shape_color attributes.

        @rtype: int
        @return: The resulting table id (Note: not image_id) of the classification row if successfully upserted, otherwise -1
        """
        insertCls = "INSERT INTO " + self.outgoingTableName
        updateCls = "UPDATE SET "

        # this will dynamically build the upsert string based on 
        # only the values that were provided us in classification
        insertValues = []
        insertClmnNames = '('
        insertClmnValues = ' VALUES('
        for clmn, value in classification.toDict(exclude=('id',)).items():
            insertClmnNames += clmn + ', '
            insertClmnValues += '%s, '
            updateCls += clmn + '= %s, '
            insertValues.append(value.__str__())

        # if there were no values to insert...
        if not insertValues:
            return -1
        else: 
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') ON CONFLICT (image_id) DO '
            updateCls = updateCls[:-2] + 'RETURNING id;'

        insertCls += insertClmnNames + insertClmnValues + updateCls
        return super(ClassificationDAO, self).getResultingId(insertCls, insertValues + insertValues)

    def addClassification(self, classification):
        """
        Adds the specified classification information to one of the outgoing tables

        @type classification: outgoing_autonomous or outgoing_manual
        @param classification: The classifications to add to the database

        @rtype: int
        @return: Id of classification if inserted, otherwise -1
        """
        # Compose the insert statement::
        insertCls = "INSERT INTO " + self.outgoingTableName

        # only inserting the values that were provided to us
        insertValues = []
        insertClmnNames = '('
        insertClmnValues = ' VALUES('
        for clmn, value in classification.toDict(exclude=('id',)).items():
            insertClmnNames += clmn + ', '
            insertClmnValues += '%s, '
            insertValues.append(value.__str__())

        # if there were no values to insert...
        if not insertValues:
            return -1
        else: 
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') RETURNING id;'

        insertCls += insertClmnNames + insertClmnValues
        return super(ClassificationDAO, self).getResultingId(insertCls, insertValues)

    def getClassificationByUID(self, id):
        """
        Attempts to get the classification with the specified universal-identifier

        @type id: int
        @param id: The id of the image to try and retrieve
        """

        selectClsById = """SELECT id, image_id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, submitted
            FROM """ + self.outgoingTableName + """ 
            WHERE image_id = %s
            LIMIT 1;"""

        selectedClass = super(ClassificationDAO, self).basicTopSelect(selectClsById, (id,))
        return selectedClass

    def getClassification(self, id):
        """
        Gets a classification by the TABLE ID.
        This is opposed to getClassificationByUID, which retrieves a row based off of the unique image_id
        This is mostly used internally, and is not used by any of the public REST API methods.

        @type id: int
        @param id: The table id of the classification to retrieve.

        @rtype: [string]
        @return: String list of values retrieved from the database. Child classes will properly place these values in model objects. If the given id
                 doesn't exist, None is returned.
        """

        selectClsById = """SELECT id, image_id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, submitted
            FROM """ + self.outgoingTableName + """ 
            WHERE id = %s
            LIMIT 1;"""
        
        selectedClass = super(ClassificationDAO, self).basicTopSelect(selectClsById, (id,))
        return selectedClass

    def getAll(self):
        """
        Get all the images currently in this table

        @rtype: cursor
        @return: A cursor to the query result for the specified classification type. This allows children
                  classes to place the results in their desired object type.
        """

        selectAllSql = """SELECT id, image_id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, submitted
            FROM """ + self.outgoingTableName + """ 
            ORDER BY id;"""

        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        # this cursor will be closed by the child
        return cur

    def updateClassificationByUID(self, id, updateClass):
        """
        Builds an update string based on the available key-value pairs in the given classification object
        if successful, returns an classification object of the entire row that was updated

        @type id: int
        @param id: The image_id of the classification to update

        @type updateClass: outgoing_autonomous or outgoing_manual
        @param updateClass: Information to attempt to update for the classification with the provided image_id

        @rtype: outgoing_autonomous or outgoing_manual
        @return: The classification of the now updated image_id if successful. Otherwise None
        """

        updateStr = "UPDATE " + self.outgoingTableName + " SET "

        values = []
        for clmn, value in updateClass.toDict().items():
            updateStr += clmn + "= %s, "
            values.append(value.__str__())

        # if someone tried to pass an empty update
        if not values:
            return None
        
        updateStr = updateStr[:-2] # remove last space/comma
        updateStr += " WHERE image_id = %s RETURNING id;"
        values.append(id)
        resultId = super(ClassificationDAO, self).getResultingId(updateStr, values)
        if resultId != -1:
            return self.getClassification(resultId)
        else:
            return None

    def getAllDistinct(self, modelGenerator, whereClause=None):
        """
        Get all the unique classifications in the classification queue
        Submitted or not.
        """
        
        # start by getting the distinct target types in our table
        # a 'distinct target' is one with unique shape and character
        getDistinctTypes = """SELECT alphanumeric, shape, type
            FROM """ + self.outgoingTableName

        selectClass = """SELECT id, image_id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, submitted
            FROM """ + self.outgoingTableName + " WHERE "

        if whereClause is not None:
            getDistinctTypes += " WHERE " + whereClause + " "
            selectClass += whereClause + " AND "
        getDistinctTypes += "GROUP BY alphanumeric, shape, type;"
        selectClass += " alphanumeric = %s and shape = %s and type = %s;"

        print(getDistinctTypes)

        distinctClassifications = []
        cur = self.conn.cursor()
        cur.execute(getDistinctTypes)

        if cur is None:
            return distinctClassifications

        classCur = self.conn.cursor()
                
        # classCur.prepare(selectClass)
        for row in cur:

            classification = []
            classCur.execute(selectClass, row)
            for result in classCur:
                outRow = modelGenerator.newModelFromRow(result)
                classification.append(outRow)

            distinctClassifications.append(classification)

        classCur.close()
        cur.close()
        return distinctClassifications
