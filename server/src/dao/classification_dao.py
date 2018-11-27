import psycopg2
from dao.base_dao import BaseDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous

class ClassificationDAO(BaseDAO):

    def __init__(self, configFilePath, outgoingTableName):
        super(ClassificationDAO, self).__init__(configFilePath)
        self.outgoingTableName = outgoingTableName # 0 for autonomous. 1 for manual

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
        """

        selectClsById = """SELECT id, image_id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, submitted
            FROM """ + self.outgoingTableName + """ 
            WHERE id = %s
            LIMIT 1;"""
        
        selectedClass = super(ClassificationDAO, self).basicTopSelect(selectClsById, (id,))
        return selectedClass

    def getAll(self):
        """
        get all the images currently in this table
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
        """

        updateStr = "UPDATE " + self.outgoingTableName + " SET "

        values = []
        for clmn, value in updateClass.toDict().items():
            updateStr += clmn + "= %s, "
            values.append(value.__str__())

        # if someone tried to pass an empty update
        if not values:
            return -1
        
        updateStr = updateStr[:-2] # remove last space/comma
        updateStr += " WHERE image_id = %s RETURNING id;"
        values.append(id)
        resultId = super(ClassificationDAO, self).getResultingId(updateStr, values)
        if resultId != -1:
            return self.getClassification(resultId)
        else:
            return -1

