from dao.base_dao import BaseDAO
from dao.model.incoming_gps import incoming_gps


class IncomingGpsDAO(BaseDAO):
    """
    Handles interaction with recorded GPS measurements. Ros_ingest interacts
    with this DAO directly. On the REST side, most of its functionality is accessed through
    the /gps endpoint
    """

    def __init__(self, configFilePath):
        super(IncomingGpsDAO, self).__init__(configFilePath)

    def addGps(self, incomingGps):
        """
        Adds the specified gps measurement to the incoming_gps table

        @type incomingGps: incoming_gps
        @param incomingGps: The gps measurement to add to the database

        @rtype: int
        @return: Id of gps measurement if successfully inserted, otherwise -1
        """
        insertStmt = "INSERT INTO incoming_gps (time_stamp, latitude, longitude, altitude) VALUES(to_timestamp(%s) AT TIME ZONE 'UTC', %s, %s, %s) RETURNING id;"
        return super(IncomingGpsDAO, self).getResultingId(insertStmt, incomingGps.insertValues())

    def getGpsById(self, id):
        """
        Get a gps measurement from the database by its id. This will likely only be 
        used internally, if at all.

        @type id: int
        @param id: The unique id of the gps measurement to retrieve

        @rtype: incoming_gps
        @returns: An incoming_gps object with all recorded gps information if the measurement
                  with the given id exists, otherwise None.
        """
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
        Get the gps row that has a time_stamp closest to the one specified.
        This will likely be the method most used by geolocation and autonomous
        localization methods.

        @type ts: float
        @param ts: UTC Unix Epoch timestamp as a float. The closest gps measurement to this timestamp will be returned

        @rtype: incoming_gps
        @return: An incoming_gps object will all the recorded gps information for the measurement
                 closest to the provided timestamp. Note that if the provided timestamp is lower than
                 all timestamp measurements or if the gps table is empty, None will be returned.
        """
        # get gps as <= to the desired value. just easier todo that then get the absolute closest
        selectGpsByTs = """SELECT id, date_part('epoch', time_stamp), latitude, longitude, altitude
            FROM incoming_gps
            WHERE incoming_gps.time_stamp <= to_timestamp(%s) AT TIME ZONE 'UTC'
            ORDER BY incoming_gps.time_stamp DESC
            LIMIT 1;"""

        selectedGps = super(IncomingGpsDAO, self).basicTopSelect(selectGpsByTs, (ts,))
        
        if selectedGps is not None:
            return incoming_gps(selectedGps)
        return None

    def getAll(self):
        """
        Get all GPS entries currently in the table

        @rtype: [incoming_gps model object]
        @return: A list of all incoming_gps entries in the table, otherwise None
        """
        selectAllSql = """SELECT id, date_part('epoch', time_stamp), latitude, longitude, altitude
            FROM incoming_gps;"""

        return super(IncomingGpsDAO, self).getResultsAsModelList(selectAllSql, None)

    def getFirst(self):
        """
        Get the first (aka oldest) GPS measurement in the table
        """
        selectOldest = """SELECT id, date_part('epoch', time_stamp), latitude, longitude, altitude
            FROM incoming_gps
            ORDER BY id ASC LIMIT 1;"""

        selectedGps = super(IncomingGpsDAO, self).basicTopSelect(selectOldest, None)

        if selectedGps is not None:
            return incoming_gps(selectedGps)
        return None

    def newModelFromRow(self, row, json=None):
        """
        Create a new incoming_gps model from the given info
        """
        return incoming_gps(row)