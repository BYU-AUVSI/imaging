from dao.base_dao import BaseDAO

class UtilDAO(BaseDAO):
    """
    Holds utility methods to help manage the database
    """

    def __init__(self, configFilePath):
        super(UtilDAO, self).__init__(configFilePath)

    def resetManualDB(self):
        """
        Resets the database to an initial form as if a rosbag
        was just read in
        """
        truncateCropped = "DELETE FROM cropped_manual;"
        truncateClassification = "DELETE FROM outgoing_manual;"
        truncateSubmitted = "DELETE FROM submitted_target;"
        updateIncoming = "UPDATE incoming_image SET manual_tap=FALSE WHERE manual_tap=TRUE;"

        super(UtilDAO, self).executeStatements([truncateClassification, truncateCropped, truncateSubmitted, updateIncoming])

    def resetAutonomousDB(self):
        """
        Resets the database to an initial form as if a rosbag
        was just read in
        """
        truncateCropped = "DELETE FROM cropped_autonomous;"
        truncateClassification = "DELETE FROM outgoing_autonomous;"
        truncateSubmitted = "DELETE FROM submitted_target;"
        updateIncoming = "UPDATE incoming_image SET autonomous_tap=FALSE WHERE autonomous_tap=TRUE;"

        super(UtilDAO, self).executeStatements([truncateClassification, truncateCropped, truncateSubmitted, updateIncoming])