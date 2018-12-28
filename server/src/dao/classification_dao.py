import psycopg2
from dao.base_dao import BaseDAO

class ClassificationDAO(BaseDAO):
    """
    Does most of the heavy lifting for classification tables: outgoing_autonomous and outgoing_manual.
    Contains general database methods which work for both types.
    """

    def __init__(self, configFilePath, outgoingTableName, uidclmn):
        super(ClassificationDAO, self).__init__(configFilePath)
        self.outgoingTableName = outgoingTableName # 0 for autonomous. 1 for manual
        # uid column is different for manual and autonomous. For manual it is the crop_id
        # which is a foreign key to the associated cropped image in the manual_cropped table
        # for autonomous, its just the id column, since its the only unique-enforced column
        # in the autonomous table. the UID column is needed for the upsert method, which requires
        # a unique-constrained column to detect insertion conflicts
        self.uidclmn = uidclmn

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
            insertClmnValues = insertClmnValues[:-2] + ') ON CONFLICT (' + self.uidclmn + ') DO '
            updateCls = updateCls[:-2] + 'RETURNING id;'

        insertCls += insertClmnNames + insertClmnValues + updateCls
        id = super(ClassificationDAO, self).getResultingId(insertCls, insertValues + insertValues)
        self.assignTargetBin(id)
        return id

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
        id = super(ClassificationDAO, self).getResultingId(insertCls, insertValues)
        self.assignTargetBin(id)
        return id

    def getClassificationByUID(self, id):
        """
        Attempts to get the classification with the specified universal-identifier

        @type id: int
        @param id: The id of the image to try and retrieve
        """
        # its best just to use * on these given the table differences between manual/autonomous
        # for autonomous, getClassification and getClassificationByUID are identical since its 
        # UID column is just 'id'
        selectClsById = """SELECT *
            FROM """ + self.outgoingTableName + """ 
            WHERE """ + self.uidclmn + """ = %s
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

        selectClsById = """SELECT *
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

        selectAllSql = """SELECT *
            FROM """ + self.outgoingTableName + """ 
            ORDER BY id;"""

        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        # this cursor will be closed by the child
        return cur

    def updateClassification(self, id, updateClass):
        """
        Builds an update string based on the available key-value pairs in the given classification object
        if successful, returns an classification object of the entire row that was updated

        @type id: int
        @param id: The id of the classification to update (ie: the id column in the classification table)

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
        updateStr += " WHERE id = %s RETURNING id;"
        values.append(id)
        resultId = super(ClassificationDAO, self).getResultingId(updateStr, values)
        self.assignTargetBin(resultId) # the update could have potentially changed the target bin for this classification
        if resultId != -1:
            return self.getClassification(resultId)
        else:
            return None

    def updateClassificationByUID(self, id, updateClass):
        """
        Builds an update string based on the available key-value pairs in the given classification object
        if successful, returns an classification object of the entire row that was updated

        @type id: int
        @param id: The uid value (id for autonomous, crop_id for manual) of the classification to update

        @type updateClass: outgoing_autonomous or outgoing_manual
        @param updateClass: Information to attempt to update for the classification with the provided image_id

        @rtype: outgoing_autonomous or outgoing_manual
        @return: The classification of the now updated uid row if successful. Otherwise None
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
        updateStr += " WHERE " + self.uidclmn + " = %s RETURNING id;"
        values.append(id)
        resultId = super(ClassificationDAO, self).getResultingId(updateStr, values)
        self.assignTargetBin(resultId) # update could have potentially changed the target bin for this classification
        if resultId != -1:
            return self.getClassification(resultId)
        else:
            return None

    def getAllDistinct(self, modelGenerator, whereClause=None):
        """
        Get all the unique classifications in the classification queue
        Submitted or not.

        TODO: redo this method given the new target clmn
        """
        
        # start by getting the distinct target types in our table
        # a 'distinct target' is one with unique shape and character
        getDistinctTypes = """SELECT alphanumeric, shape, type
            FROM """ + self.outgoingTableName

        selectClass = """SELECT *
            FROM """ + self.outgoingTableName + " WHERE "

        if whereClause is not None:
            getDistinctTypes += " WHERE " + whereClause + " "
            selectClass += whereClause + " AND "
        getDistinctTypes += " GROUP BY alphanumeric, shape, type;"
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

    def getAllTargets(self, modelGenerator, whereClause=None):
        return None

    def assignTargetBin(self, id):
        """
        Internal method used for determinging what target bin a particular row (specified
        by the UID param should belong to)

        @type id: int
        @param id: the base classification id of the classification to bin into a target (aka the id column)

        @return: None
        """
        #NOTE: This select needs to work for either 'outgoing_' table
        getClassificationInfo = """SELECT id, alphanumeric, shape, type 
            FROM """ + self.outgoingTableName + """
            WHERE id = %s;"""

        classificationInfo = super(ClassificationDAO, self).basicTopSelect(getClassificationInfo, (id,))
        if classificationInfo is None or not classificationInfo:
            print("Failed to retrieve info for uid {}".format(id))
            return

        # use the retrieved info to see if there are other targets 
        # that match and what target bin they are

        # basically we'll assign to the first matching target (limit 1), since
        # this whole setup assumes that other matching targets would have the 
        # same target id
        getPotentialTargetIds = """SELECT target 
            FROM """ + self.outgoingTableName + """ 
            WHERE (alphanumeric = %s OR (alphanumeric IS NULL AND %s IS NULL)) 
                and (shape = %s OR (shape IS NULL AND %s IS NULL)) 
                and (type  = %s OR (type  IS NULL AND %s IS NULL)) LIMIT 1;"""
        # all this complicated stuff is for if one of these columns is NULL

        cur = self.conn.cursor()
        cur.execute(getPotentialTargetIds, (classificationInfo[1], classificationInfo[1], classificationInfo[2], classificationInfo[2], classificationInfo[3], classificationInfo[3]))

        if cur is None:
            print("Something went wrong trying to get target id for {}".format(id))
            return

        # default target id to indicate there is no target id for
        # this type yet
        targetId = -1

        try:
            targetIdResult = cur.fetchone()
            # if there's a target this uid belongs to, grab it
            if targetIdResult is not None and targetIdResult[0] is not None:
                targetId = targetIdResult[0]
        except (Exception) as error:
            print("No result for potential target id for uid {}".format(id))

        if targetId == -1:
            # then we need to figure out what target id we can assign it to
            # grab the current largest target id in the table
            getLargestTargetId = "SELECT MAX(target) FROM " + self.outgoingTableName

            largestTargetId = super(ClassificationDAO, self).basicTopSelect(getLargestTargetId, None)
            
            if largestTargetId is None or largestTargetId[0] is None:
                # then there are currently no targets in the table.
                # this likely indicates that the current uid is the very fist to be added
                # just start target counting at 1
                targetId = 1
            else:
                # if there are target ids, we know from the previous query's failure that the current uid
                # represents a new target. reflect this by adding one to the current largest id
                targetId = largestTargetId[0] + 1
        
        updateTargetId = "UPDATE " + self.outgoingTableName + " SET target = %s WHERE id = %s RETURNING id;"

        ret = super(ClassificationDAO, self).getResultingId(updateTargetId, (targetId, id))
        if ret == -1:
            print("Updating target id column for {} failed!".format(id))
