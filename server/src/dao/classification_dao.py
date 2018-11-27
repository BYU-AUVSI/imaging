import psycopg2
from dao.base_dao import BaseDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous

class classificationDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(classificationDAO, self).__init__(configFilePath)
        self.classificationTableType; # 0 for autonomous. 1 for manual

    def addImage(self, classification):
        """
        Adds the specified classification iformation to one of the outgoing tables
        @type classification: outgoing_autonomous or outgoing_manual
        @param classification: The classifications to add to the database

        @rtype: int
        @return: Id of classification if inserted, otherwise -1
        """


        insertCls = "INSERT INTO "
        if(self.classificationTableType):
            insertCls += "outgoing_manual\n"
        else:
            insertCls += "outgoing_autonomous\n"

        insertCls += """(id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, cropped_path, submitted)
        VALUES(%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, )
        RETURNING id;"""

        return super(classificationDAO, self).getResultingId(insertCls, manualCropped.insertValues())


    def getImageByUID(self, id):
        """
        Attempts to get the classification with the specified universal-identifier

        @type id: int
        @param id: The id of the image to try and retrieve

        @rtype: manual_cropped
        @return: A manual_cropped image with the info for that image if successfully found, otherwise None
        """

        selectClsById = """SELECT id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, cropped_path, submitted
        FROM """
        if(self.classificationTableType):
            selectClsById += "outgoing_manual\n"
        else:
            selectClsById += "outgoing_autonomous\n"

        selectClsById += """WHERE image_id = %s
        LIMIT 1;"""

            selectedImage = super(ManualCroppedDAO, self).basicTopSelect(selectClsById, (id,))

        if selectedImage is None:
            return None
        return manual_cropped(selectedImage)


    def getAll(self):
        """
        get all the images currently in this table
        """

        selectAllSql = """SELECT id, type, latitude, longitude, orientation, shape, background_color, alphanumeric, alphanumeric_color, description, cropped_path, submitted
        FROM """

        if(self.classificationTableType):
            selectAllSql += "outgoing_manual\n"
        else:
            selectAllSql += "outgoing_autonomous\n"

        selectAllSql += "ORDER BY id;"

        cur = self.conn.cursor()
        cur.execute(selectAllSql)
        results = []
        for row in cur:
            if(self.classificationTableType):
                outgoingAutonomousRow = outgoing_autonomous(row)
                results.append(outgoingAutonomousRow)
            else:
                outgoingManualRow = outgoing_manual(row)
                results.append(outgoingManualRow)

        return results
