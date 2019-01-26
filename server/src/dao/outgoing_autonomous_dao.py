import psycopg2
from dao.classification_dao import ClassificationDAO
from dao.model.outgoing_autonomous import outgoing_autonomous

class OutgoingAutonomousDAO(ClassificationDAO):
    """
    Outgoing_autonomous wrapper for the ClassificationDAO. Most of the core
    functionality here happens in the ClassificationDAO
    """

    def __init__(self, configFilePath):
        super(OutgoingAutonomousDAO, self).__init__(configFilePath, 'outgoing_autonomous', newModelFromRow)

    def getPendingTargets(self):
        """
        See classification_dao docs
        Get images grouped by distinct targets pending submission (ei: submitted = false)
        """
        return super(OutgoingAutonomousDAO, self).getAllTargets(whereClause=" submitted = 'unsubmitted' ")


def newModelFromRow(row, json=None):
    """
    Create a new outgoing_autonomous model object given a list of sql values from the table,
    or a json dictionary

    @type row: [string]
    @param row: List of ordered string values to be placed within an outgoing_autonomous object
    """
    return outgoing_autonomous(row, json=json)