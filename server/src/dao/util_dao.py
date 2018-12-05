from dao.base_dao import BaseDAO

class UtilDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(UtilDAO, self).__init__(configFilePath)

    def resetDB(self):
        """
        Resets the database to an inital form as if a rosbag
        was just read in
        """

        truncateCropped = "DELETE FROM manual_cropped;"
        truncateClassification = "DELETE FROM outgoing_manual;"
        updateIncoming = "UPDATE incoming_image SET manual_tap=FALSE WHERE manual_tap=TRUE;"

        super(UtilDAO, self).executeStatements([truncateClassification, truncateCropped, updateIncoming])