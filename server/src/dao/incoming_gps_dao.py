from base_dao import BaseDAO
from model.incoming_gps import incoming_gps


class IncomingGpsDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(IncomingGpsDAO, self).__init__(configFilePath)

    def addGps(self, incomingGps):
        """
        Adds the specified image to the incoming_image table
        @type incomingGps: incoming_gps
        @param incomingGps: The gps measurement to add to the database

        @rtype: int
        @return: Id of gps measurement if successfully inserted, otherwise -1
        """
        insertStmt = "INSERT INTO incoming_gps (time_stamp, latitude, longitude, altitude) VALUES(to_timestamp(%s), %s, %s, %s) RETURNING id;"
        return super(IncomingGpsDAO, self).basicInsert(insertStmt, incomingGps.insertValues())