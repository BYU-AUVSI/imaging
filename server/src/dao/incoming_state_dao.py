from dao.base_dao import BaseDAO
from dao.model.incoming_state import incoming_state


class IncomingStateDAO(BaseDAO):
    """
    Handles interaction with recorded state measurements. Ros_ingest interacts
    with this DAO directly. On the REST side, most of its functionality is accessed through
    the /state endpoint
    """

    def __init__(self, configFilePath):
        super(IncomingStateDAO, self).__init__(configFilePath)

    def addState(self, incomingState):
        """
        Adds the specified image to the incoming_image table
        
        @type incomingState: incoming_state
        @param incomingState: The state measurements to add to the database

        @rtype: int
        @return: Id of state measurements if successfully inserted, otherwise -1
        """
        insertStmt = "INSERT INTO incoming_state (time_stamp, roll, pitch, yaw) VALUES(to_timestamp(%s), %s, %s, %s) RETURNING id;"
        return super(IncomingStateDAO, self).getResultingId(insertStmt, incomingState.insertValues())

    def getStateById(self, id):
        """
        Get a state measurement from the database by its id. This will likely only be 
        used internally, if at all.

        @type id: int
        @param id: The unique id of the state measurement to retrieve

        @rtype: incoming_state
        @returns: An incoming_state object with all recorded state information if the measurement
                  with the given id exists, otherwise None.
        """
        # note we need the date_part to convert the time_stamp back to unix epoch time
        selectStateById = """SELECT id, date_part('epoch', time_stamp), roll, pitch, yaw
            FROM incoming_State
            WHERE id = %s LIMIT 1;"""
        selectedState = super(IncomingStateDAO, self).basicTopSelect(selectStateById, (id,))

        if selectedState is not None:
            return incoming_state(selectedState)
        return None

    def getStateByClosestTS(self, ts):
        """
        Get the state row that has a time_stamp closest to the one specified.
        This will likely be the method most used by geolocation and autonomous
        localization methods.

        @type ts: float
        @param ts: UTC Unix Epoch timestamp as a float. The closest state measurement to this timestamp will be returned

        @rtype: incoming_state
        @return: An incoming_state object will all the recorded state information for the measurement
                 closest to the provided timestamp. Note that if the provided timestamp is lower than
                 all timestamp measurements or if the state table is empty, None will be returned.
        """
        # test val:: 1541063324.1
        # get gps as <= to the desired value. just easier todo that then get the absolute closest
        selectGpsByTs = """SELECT id, date_part('epoch', time_stamp), roll, pitch, yaw
            FROM incoming_state
            WHERE incoming_state.time_stamp <= to_timestamp(%s) AT TIME ZONE 'UTC'
            ORDER BY incoming_state.time_stamp DESC
            LIMIT 1;"""

        selectedState = super(IncomingStateDAO, self).basicTopSelect(selectGpsByTs, (ts,))

        if selectedState is not None:
            return incoming_state(selectedState)
        return None
