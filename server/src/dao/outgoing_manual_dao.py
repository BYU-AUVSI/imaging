import psycopg2
from dao.classification_dao import ClassificationDAO
from dao.model.outgoing_manual import outgoing_manual

class OutgoingManualDAO(ClassificationDAO):
    """
    Outgoing_manual wrapper for the ClassificationDAO. Most of the core
    functionality here happens in the ClassificationDAO
    """

    def __init__(self, configFilePath):
        super(OutgoingManualDAO, self).__init__(configFilePath, 'outgoing_manual', newModelFromRow)
    
    def getPendingTargets(self):
        """
        See classification_dao docs
        Get images grouped by distinct targets pending submission (ei: submitted = 'unsubmitted')
        """
        return super(OutgoingManualDAO, self).getAllTargets(self, whereClause=" submitted = 'unsubmitted' ")

def newModelFromRow(row, json=None):
    """
    Create a new outgoing_manual model object given a list of sql values from the table,
    or a json dictionary

    @type row: [string]
    @param row: List of ordered string values to be placed within an outgoing_manual object
    """
    return outgoing_manual(row, json=json)