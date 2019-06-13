from dao.base_dao import BaseDAO
from config import getLatestBaseImgDir, createNewBaseImgDir
import os

class UtilDAO(BaseDAO):
    """
    Holds utility methods to help manage the database
    """

    def __init__(self, configFilePath):
        super(UtilDAO, self).__init__(configFilePath)

    tables = ['incoming_image', 'incoming_gps', 'incoming_state', 
        'cropped_manual', 'cropped_autonomous', 'outgoing_manual', 
        'outgoing_autonomous', 'submitted_target']
    deleteSQL = "DELETE FROM {};"
    csvExportSQL = "COPY (SELECT * FROM {}) TO STDOUT WITH CSV DELIMITER ';';"
    # csvExportSQL = "\copy (SELECT * FROM {}) TO '{}' WITH CSV;"
    truncateIncomingManual = "DELETE FROM incoming_image;"
    truncateCroppedManual = "DELETE FROM cropped_manual;"
    truncateCroppedAuto = "DELETE FROM cropped_autonomous;"
    truncateClassificationManual = "DELETE FROM outgoing_manual;"
    truncateClassificationAuto = "DELETE FROM outgoing_autonomous;"
    truncateSubmitted = "DELETE FROM submitted_target;"

    def resetManualDB(self):
        """
        Resets the database to an initial form as if a rosbag
        was just read in
        """
        
        updateIncoming = "UPDATE incoming_image SET manual_tap=FALSE WHERE manual_tap=TRUE;"

        super(UtilDAO, self).executeStatements([self.truncateClassificationManual, self.truncateCroppedManual, self.truncateSubmitted, updateIncoming])

    def resetAutonomousDB(self):
        """
        Resets the database to an initial form as if a rosbag
        was just read in
        """
        updateIncoming = "UPDATE incoming_image SET autonomous_tap=FALSE WHERE autonomous_tap=TRUE;"

        super(UtilDAO, self).executeStatements([self.truncateClassificationAuto, self.truncateCroppedAuto, self.truncateSubmitted, updateIncoming])

    def resetAll(self):
        """
        Truncates ALL the tables in the AUVSI database, reseting them to their 
        empty state.
        NOTE: this will not delete any of the images that the tables themselves 
        are referencing on your filesystem
        """
        super(UtilDAO, self).executeStatements([self.deleteSQL.format(table) for table in self.tables])
        # after truncating all the tables create a new directory for any future images
        #   to go into as part of the new database. the ros_handler will pick up on 
        #   this new directory and use it
        createNewBaseImgDir()

    def saveAll(self):
        """
        Saves the current database state to a set of csv files located in the base
        image path folder (ie: the root directory where all the image currently)
        in the database are located
        After exporting the current database state, all tables are truncated
        and you're left with a clean database
        """
        # save all the tables:

        for table in self.tables:
            cur = self.conn.cursor()
            filename = os.path.join(getLatestBaseImgDir(), table)
            with open(filename, "w") as file:
                cur.copy_expert(self.csvExportSQL.format(table), file)
            cur.close()