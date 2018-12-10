import psycopg2
from dao.classification_dao import ClassificationDAO
from dao.model.outgoing_autonomous import outgoing_autonomous

class OutgoingAutonomousDAO(ClassificationDAO):

    def __init__(self, configFilePath):
        super(OutgoingAutonomousDAO, self).__init__(configFilePath, 'outgoing_autonomous')

    def checkedReturn(self, rawResponse):
        if rawResponse is None:
            return None

        return outgoing_autonomous(rawResponse)

    def getClassificationByUID(self, id):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type
        """
        selectedClass = super(OutgoingAutonomousDAO, self).getClassificationByUID(id)
        return self.checkedReturn(selectedClass)

    def getAll(self):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type
        """
        cur = super(OutgoingAutonomousDAO, self).getAll()

        results = []
        if cur is not None:
            for row in cur:
                outManualRow = outgoing_autonomous(row)
                results.append(outManualRow)

            cur.close() # dont forget to close the cursor
        
        return results

    def getClassification(self, id):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type
        """
        selectedClass = super(OutgoingAutonomousDAO, self).getClassification(id)
        return self.checkedReturn(selectedClass)

    def updateClassificationByUID(self, id, updateClass):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type. We're also properly setting up the 
        initial model of stuff to update before passing it to super
        """

        updateModel = outgoing_autonomous(json=updateClass)
        selectedClass = super(OutgoingAutonomousDAO, self).updateClassificationByUID(id, updateModel)
        return selectedClass

    def getAllDistinct(self):
        return super(OutgoingAutonomousDAO, self).getAllDistinct(self)

    def getAllDistinctPending(self):
        """
        Get images grouped by distinct targets pending submission (ei: submitted = false)
        """
        return super(OutgoingAutonomousDAO, self).getAllDistinct(self, whereClause="WHERE submitted=FALSE ")

    def newModelFromRow(self, row):
        """
        A reflective function for the classification dao. Pass self up
        to the super ClassificationDAO. It calls this method to create the 
        proper model object in its response. Not uber elegant, but presently used by getAllDistinct.

        @type row: [string]
        @param row: List of ordered string values to be placed within an outgoing_autonomous object
        """
        return outgoing_autonomous(row)