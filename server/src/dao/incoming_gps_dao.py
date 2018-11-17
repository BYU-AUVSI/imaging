from dao.base_dao import BaseDAO
from dao.model.incoming_gps import incoming_gps


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

    def getGpsById(self, id):
        # note we need the date_part to convert the time_stamp back to unix epoch time
        selectGpsById = """SELECT id, date_part('epoch', time_stamp), latitude, longitude, altitude 
            FROM incoming_gps 
            WHERE id = %s LIMIT 1;"""
        selectedGps = super(IncomingGpsDAO, self).basicTopSelect(selectGpsById, (id,))

        if selectedGps is not None:
            return incoming_gps(selectedGps)
        return None

    def getGpsByClosestTS(self, ts):
        """
        Get the gps row that has a time_stamp closest to the one specified
        """
        # test val:: 1541063324
        selectGpsByTs = """SELECT id, date_part('epoch', time_stamp), latitude, longitude, altitude
            FROM incoming_gps
            ORDER BY abs(extract(epoch from time_stamp - to_timestamp(%s)))
            LIMIT 1;"""

        selectedGps = super(IncomingGpsDAO, self).basicTopSelect(selectGpsByTs, (ts,))
        
        if selectedGps is not None:
            return incoming_gps(selectedGps)
        return None