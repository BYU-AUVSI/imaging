import psycopg2
from dao.base_dao import BaseDAO

class ClassificationDAO(BaseDAO):
    """
    Does most of the heavy lifting for classification tables: outgoing_autonomous and outgoing_manual.
    Contains general database methods which work for both types.
    """

    def __init__(self, configFilePath, outgoingTableName):
        super(ClassificationDAO, self).__init__(configFilePath)
        self.outgoingTableName = outgoingTableName 

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
        for clmn, value in classification.toDict(exclude=('class_id',)).items():
            insertClmnNames += clmn + ', '
            insertClmnValues += '%s, '
            updateCls += clmn + '= %s, '
            insertValues.append(value.__str__())

        # if there were no values to insert...
        if not insertValues:
            return -1
        else: 
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') ON CONFLICT (crop_id) DO '
            updateCls = updateCls[:-2] + 'RETURNING class_id;'

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
        for clmn, value in classification.toDict(exclude=('class_id',)).items():
            insertClmnNames += clmn + ', '
            insertClmnValues += '%s, '
            insertValues.append(value.__str__())

        # if there were no values to insert...
        if not insertValues:
            return -1
        else: 
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') RETURNING class_id;'

        insertCls += insertClmnNames + insertClmnValues

        id = super(ClassificationDAO, self).getResultingId(insertCls, insertValues)
        if id  != -1:
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
            WHERE crop_id = %s
            LIMIT 1;"""

        selectedClass = super(ClassificationDAO, self).basicTopSelect(selectClsById, (id,))
        if selectedClass is None:
            return None
        return self.newModelFromRow(selectedClass)

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
            WHERE class_id = %s
            LIMIT 1;"""
        
        selectedClass = super(ClassificationDAO, self).basicTopSelect(selectClsById, (id,))
        if selectedClass is None:
            return None
        return self.newModelFromRow(selectedClass)

    def removeClassification(self, id):
        """
        Remove the classification with the specified TABLE id from the tabl.
        This is useful if a classification is painfully wrong, and you dont want it
        to have any effect on the final submission.

        @type id: int
        @param id: The table id of the classification to retrieve.

        @rtype: boolean
        @return: True if the classification was successfully removed, otherwise False
        """
        removeSql = "DELETE FROM " + self.outgoingTableName + " WHERE class_id = %s;"

        rcount = super(ClassificationDAO, self).getNumAffectedRows(removeSql, (id,))

        return rcount > 0

    def getAll(self):
        """
        Get all the images currently in this table

        @rtype: cursor
        @return: A cursor to the query result for the specified classification type. This allows children
                  classes to place the results in their desired object type.
        """

        selectAllSql = """SELECT *
            FROM """ + self.outgoingTableName + """ 
            ORDER BY class_id;"""

        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        
        results = []
        if cur is not None:
            for row in cur:
                outModel = self.newModelFromRow(row)
                results.append(outModel)

        return results

    def updateClassification(self, id, updateJson):
        """
        Builds an update string based on the available key-value pairs in the given classification object
        if successful, returns an classification object of the entire row that was updated

        @type id: int
        @param id: The id of the classification to update (ie: the id column in the classification table)

        @type updateJson: json dictionary
        @param updateJson: Information to attempt to update for the classification with the provided class_id

        @rtype: outgoing_autonomous or outgoing_manual
        @return: The classification of the now updated class_id if successful. Otherwise None
        """
        updateStr = "UPDATE " + self.outgoingTableName + " SET "

        values = []
        for clmn, value in updateJson.items():
            updateStr += clmn + "= %s, "
            values.append(value.__str__())

        # if someone tried to pass an empty update
        if not values:
            return None
        
        updateStr = updateStr[:-2] # remove last space/comma
        updateStr += " WHERE class_id = %s RETURNING class_id;"
        values.append(id)
        resultId = super(ClassificationDAO, self).getResultingId(updateStr, values)
        self.assignTargetBin(resultId) # the update could have potentially changed the target bin for this classification
        if resultId != -1:
            return self.getClassification(resultId)
        else:
            return None

    def updateClassificationByUID(self, crop_id, updateJson):
        """
        Builds an update string based on the available key-value pairs in the given classification object
        if successful, returns an classification object of the entire row that was updated

        @type crop_id: int
        @param crop_id: The uid value (crop_id) of the classification to update

        @type updateJson: json dictionary
        @param updateJson: Information to attempt to update for the classification with the provided class_id

        @rtype: outgoing_autonomous or outgoing_manual
        @return: The classification of the now updated uid row if successful. Otherwise None
        """
        if updateJson is None:
            return None

        updateStr = "UPDATE " + self.outgoingTableName + " SET "

        values = []
        for clmn, value in updateJson.items():
            updateStr += clmn + "= %s, "
            values.append(value.__str__())

        # if someone tried to pass an empty update
        if not values:
            return None
        
        updateStr = updateStr[:-2] # remove last space/comma
        updateStr += " WHERE crop_id = %s RETURNING class_id;"
        values.append(crop_id)
        resultId = super(ClassificationDAO, self).getResultingId(updateStr, values)
        self.assignTargetBin(resultId) # update could have potentially changed the target bin for this classification
        if resultId != -1:
            return self.getClassification(resultId)
        else:
            return None

    def getAllTargets(self, whereClause=None):
        """
        Get all the unique classifications in the classification queue
        Submitted or not.

        TODO: redo this method given the new target clmn
        """
        # get the target ids that work with our where clause:
        targets = self.getAllTargetIDs(whereClause)

        if targets is None or not targets:
            return []

        # NOTE: this is a bit dangerous, but allows this query to work on either
        # table. We're RELYING ON the target column being at INDEX 2
        getAll = """SELECT *
            FROM """ + self.outgoingTableName + """
            WHERE target IN %s ORDER BY target;"""

        cur = self.conn.cursor()
        cur.execute(cur.mogrify(getAll, (tuple(targets),)))

        rawRecords = cur.fetchall()

        allClassifications = []
        lastTarget = -1

        currentTarget = []

        for record in rawRecords:
            model = self.newModelFromRow(record)

            if record[2] != lastTarget and lastTarget != -1:
                # ie: if the target has changed
                allClassifications.append(currentTarget)
                currentTarget = []

            currentTarget.append(model)
            lastTarget = record[2]

        allClassifications.append(currentTarget)

        return allClassifications

    def getAllTargetIDs(self, whereClause=None):
        """
        Get a list of the target ids currently in this outgoing table.

        @type whereClause: string
        @param whereClause: Specify a WHERE to select the targets on. Essentially apply a filter
            for what target ids are returned. ie: pass "submitted = 'unsubmitted'" for this method
            to return a list of target ids that have not been submitted yet.
        
        @rtype: list of int
        @return: A list of all target ids if no whereClause is provided and the query is 
            successful. Or a list of filtered ids if a whereClause is provided. If something fails,
            None
        """
        # build our sql query
        getTargetIds = """SELECT target
            FROM """ + self.outgoingTableName

        # add the where clause if we have one
        if whereClause is not None:
            getTargetIds += " WHERE " + whereClause

        getTargetIds += " GROUP BY target;"

        cur = self.conn.cursor()
        cur.execute(getTargetIds)

        if cur is None:
            cur.close()
            return None

        targetList = []
        rawRecords = cur.fetchall() #list of list

        for record in rawRecords:
            targetList.append(record[0])

        return targetList

    def isTargetSubmitted(self, target):
        """
        Check if the specified target has already been submitted

        @rtype: Boolean
        @return: True if the target has been submitted, otherwise False
        """
        if target is None:
            return False

        getSubmitted = """SELECT target, submitted
            FROM """ + self.outgoingTableName + """
            WHERE target = %s AND submitted = 'submitted' LIMIT 1;"""

        submittedTarget = super(ClassificationDAO, self).basicTopSelect(getSubmitted, (target,))
        
        if submittedTarget is None or not submittedTarget:
            return False
        elif submittedTarget[0] == target:
            return True # we got a row

        return False

    def submitAllPendingTargets(self):
        """
        Finds all targets with a 'unsubmitted' status and submits them

        @rtype: [outgoing_manual] or [outgoing_autonomous] model
        @return: A list of classifications that should be submitted to the
            judges server. Note that it's the job of higher-level DAOs/handlers
            to make sure this information is properly submitted. None if error
            or no ids to submit
        """

        unsubmittedTargetIds = self.getAllTargetIDs(" submitted = 'unsubmitted' ")

        if unsubmittedTargetIds is None or not unsubmittedTargetIds:
            print('Either something went wrong, or there are no unsubmitted target ids.')
            return None
        
        allSubmitted = []

        for targetId in  unsubmittedTargetIds:
            resultModel = self.submitPendingTarget(targetId)
            if resultModel is not None:
                allSubmitted.append(resultModel)
        
        if not allSubmitted:
            print('Failed to submit all the targets in this list: {}'.format(unsubmittedTargetIds))
            return None

        return allSubmitted

    def submitPendingTarget(self, target, optionalSpecs=None):
        """
        Changes the status to the given target id to 'submitted'. All classifications
        that are apart of the target bin will change status to 'submitted'. Any 
        classifications added to this target after the fact will then receive an
        'inherited_submission' status

        @type target: int
        @param target: Id of the target to submit (number from the 'target' column)

        @rtype: outgoing_manual or outgoing_autonomous model
        @return: a model object of what should be submitted to the judge's server
            Its the job of higher-level DAO's/ handlers to make sure this information
            is properly submitted. None if error, or if the id has already been submitted
        """

        # grab a classification for this target and set it as the one we're submitting
        claimClassificationToSubmit = 'UPDATE ' + self.outgoingTableName + """
            SET submitted = 'submitted' 
            WHERE class_id = (
                SELECT class_id FROM """ + self.outgoingTableName + """ 
                WHERE target = %s AND submitted = 'unsubmitted' LIMIT 1)
            RETURNING class_id;"""

        # update all the other classifications for this target to be inherited_submission
        updateTargetClassifications =  'UPDATE ' + self.outgoingTableName + """
            SET submitted = 'submitted'
            WHERE target = %s AND submitted = 'unsubmitted';"""

        claimedClassification = super(ClassificationDAO, self).getResultingId(claimClassificationToSubmit, (target,))

        if claimedClassification == -1:
            # claiming a single classification failed. Likely indicating either 
            # an sql error, or that the specified target doesn't exist
            return None

        cur = self.conn.cursor()
        cur.execute(updateTargetClassifications, (target,))
        cur.close()

        return self.createTargetToSubmitFromClassifications(target, optionalSpecs)

    def createTargetToSubmitFromClassifications(self, target, optionalSpecs=None):
        """
        Get the classification that was submitted for this target.

        Note, this will perform all the averaging functions again (of lat-long and color) as
        if happening for the first time. So if another classification has been added
        for this target, final values will be slightly different
        """

        getSubmittedClass = 'SELECT * FROM ' + self.outgoingTableName + """
            WHERE target = %s AND submitted = 'submitted' LIMIT 1;"""

        # get all the classifications for the target so we can do averaging/ most-common-value stuff
        # NOTE: we use the column numbers from this query below when doing avg/MCV stuff
        getOtherClass = """SELECT class_id, crop_id, latitude, longitude, orientation, background_color, alphanumeric_color, description
            FROM """ + self.outgoingTableName + """
            WHERE target = %s;"""

        result = super(ClassificationDAO, self).basicTopSelect(getSubmittedClass, (target,))

        if result is None:
            print('Failed to retrieve submitted classification for target {}'.format(target))
            return None
        
        finalModel = self.newModelFromRow(result)

        cur = self.conn.cursor()
        cur.execute(getOtherClass, (target,))
        allClass = cur.fetchall() # list of lists. each inner list is a result row from the sql query

        if allClass is None or not allClass or not allClass[0]:
            # this should never happen
            print('Failed to retrieve all classifications for target {}'.format(target))
            return finalModel

        # always calc avg for lat long (cant manually specify classification values for them)
        finalModel.latitude           = self.calcClmnAvg(allClass, 2)
        finalModel.longitude          = self.calcClmnAvg(allClass, 3)
        if optionalSpecs is None:
            # get most common for these, if we dont have manually specified classifications to use
            finalModel.orientation        = self.findMostCommonValue(allClass, 4)
            finalModel.background_color   = self.findMostCommonValue(allClass, 5)
            finalModel.alphanumeric_color = self.findMostCommonValue(allClass, 6)
        else:
            # go through and figure out which values were manually specified
            # everything in this else can only happen for a manual target submission
            if 'crop_id' in optionalSpecs and optionalSpecs['crop_id'] is not None:
                cropVal = self.getValueWithId(allClass, optionalSpecs['crop_id'], 1)
                if cropVal is not None:
                    # NOTE: the use of crop_id. Since this section will only ever be run by
                    # manual, it's safe todo this. If you for some reason use this block with 
                    # autonomous in the future, this would be unsafe (though i cant see a reason why)
                    finalModel.crop_id = cropVal

            if 'description' in optionalSpecs and optionalSpecs['description'] is not None:
                descVal = self.getValueWithId(allClass, optionalSpecs['description'], 7)
                if descVal is not None:
                    finalModel.description = descVal
                       
            keys = ['orientation', 'background_color', 'alphanumeric_color']
            for i in range(len(keys)):
                sqlClmn = i + 4 # add offset so we know the clmn index for the all class list above
                if keys[i] in optionalSpecs and optionalSpecs[keys[i]] is not None:
                    # get it
                    val = self.getValueWithId(allClass, optionalSpecs[keys[i]], sqlClmn)
                    if val is not None:
                        # it worked! final output target submission will use the exact value from
                        # the classification_id specified in optionalSpecs
                        setattr(finalModel, keys[i], val)
                    else:
                        # if it failed to retrieve, then default to most common
                        setattr(finalModel, keys[i], self.findMostCommonValue(allClass, sqlClmn))
                else:
                    # do most common as usual
                    setattr(finalModel, keys[i], self.findMostCommonValue(allClass, sqlClmn))

        return finalModel

    def resetTargetSubmissionStatus(self, target):
        """
        Resets all classifications for the given target to an 'unsubmitted' state.
        This is useful if something at a higher level fails and the target in fact failed
        to submit

        @type target: int
        @param target: the target_id to reset
        """

        resetTargetSql = "UPDATE " + self.outgoingTableName + """
            SET submitted = 'unsubmitted' 
            WHERE target = %s;"""

        cur = self.conn.cursor()
        cur.execute(resetTargetSql, (target,))
        cur.close()      

    def calcClmnAvg(self, classifications, clmnNum):
        """
        Calculate the average of the specified column

        @type classifications: list of value lists (ie: from a cursor.fetchall())
        @param classifications: database rows to use to calculate the average
        @type clmnNun: int
        @param clmnNum: Integer of the column to access in each row to get values for avg calculation

        @rtype: float
        @return: Average of the numbers in clmnNum as a float
        """

        ttl = 0.0
        for row in classifications:
            if row[clmnNum] is not None:
                ttl += row[clmnNum]
        
        if ttl == 0.0:
            return None
        return ttl / float(len(classifications))

    def findMostCommonValue(self, classifications, clmnNum):
        """
        Calculate the most common value in a specified column (useful for all the enum columns)

        @type classifications: list of value lists (ie: from a cursor.fetchall())
        @param classifications: database rows to use to calculate the average
        @type clmnNun: int
        @param clmnNum: Integer of the column to access in each row to get values for avg calculation
        """


        # dictionary keeps track of how many times we've seen a particular column value
        # EX: {
        #       "red": 2,
        #        "white": 1}
        valueCounts = {}
        # for each classification in our list of classifications
        # a classification here is a list
        for row in classifications:
            if row[clmnNum] is not None:
                # if the value at the classification has not been added to our dictionary yet
                if row[clmnNum] not in valueCounts:
                    valueCounts[row[clmnNum]] = 0
                valueCounts[row[clmnNum]] += 1

        if valueCounts: # if the dictionary isnt empty
            mostCommon = max(valueCounts, key=valueCounts.get)
            return mostCommon
        return None

    def getValueWithId(self, classifications, classificationId, clmnToRetrieve):
        try:
            # bit of input validation
            classificationId = int(classificationId)
        except (Exception) as error:
            print(error)
            return None
    
        for classification in classifications:
            if classification[0] == classificationId:
                if classification[clmnToRetrieve] is not None:
                    return classification[clmnToRetrieve]
                else:
                    return None
        return None
            
    def assignTargetBin(self, id):
        """
        Internal method used for determinging what target bin a particular 
        classification/row (specified by the UID param) should belong to.

        A classification's alphanumeric, shape and type determine what target
        it belongs to.

        @type id: int
        @param id: the base classification id of the classification to bin into a target (aka the id column)

        @return: None
        """
        #NOTE: This select needs to work for either 'outgoing_' table
        getClassificationInfo = """SELECT class_id, alphanumeric, shape, type 
            FROM """ + self.outgoingTableName + """
            WHERE class_id = %s LIMIT 1;"""

        # get this classification's alphanumeric, shape and type. We use these
        # to specify what a target is
        classificationInfo = super(ClassificationDAO, self).basicTopSelect(getClassificationInfo, (id,))
        if classificationInfo is None or not classificationInfo:
            print("Failed to retrieve info for uid {}".format(id))
            return

        # use the retrieved info to see if there are other targets 
        # that match and what target bin they are

        # basically we'll assign to the first matching target (limit 1), since
        # this whole setup assumes that other matching targets would have the 
        # same target id
        getPotentialTargetIds = """SELECT target, submitted 
            FROM """ + self.outgoingTableName + """ 
            WHERE (alphanumeric = %s OR (alphanumeric IS NULL AND %s IS NULL)) 
                AND (shape = %s OR (shape IS NULL AND %s IS NULL)) 
                AND (type  = %s OR (type  IS NULL AND %s IS NULL)) 
                AND class_id != %s LIMIT 1;"""
        # all this complicated WHERE clause stuff is for if one of these columns is NULL

        cur = self.conn.cursor()
        cur.execute(getPotentialTargetIds, (classificationInfo[1], classificationInfo[1], classificationInfo[2], classificationInfo[2], classificationInfo[3], classificationInfo[3], id))

        if cur is None:
            print("Something went wrong trying to get target id for {}".format(id))
            cur.close()
            return

        # default target id to indicate there is no target id for
        # this type yet
        targetId = -1
        submissionState = 'unsubmitted'

        try:
            targetIdResult = cur.fetchone()
            # if there's a target this uid belongs to, grab it
            if targetIdResult is not None and targetIdResult[0] is not None:
                targetId = targetIdResult[0]
                # check what the submission status for this target is, so that this
                # id can reflect it:
                if targetIdResult[1] != 'unsubmitted':
                    # then this target type has been submitted. make this id reflect that
                    submissionState = 'inherited_submission'
        except (Exception) as error:
            print("No result for potential target id for uid {}".format(id))

        cur.close()

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
        
        updateTargetId = "UPDATE " + self.outgoingTableName + " SET target = %s, submitted = %s WHERE class_id = %s RETURNING class_id;"

        ret = super(ClassificationDAO, self).getResultingId(updateTargetId, (targetId, submissionState, id))
        if ret == -1:
            print("Updating target id column for {} failed!".format(id))
