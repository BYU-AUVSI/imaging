import psycopg2
from dao.classification_dao import ClassificationDAO
from dao.model.outgoing_manual import outgoing_manual

class OutgoingManualDAO(ClassificationDAO):

    def __init__(self, configFilePath):
        super(OutgoingManualDAO, self).__init__(configFilePath, 'outgoing_manual')


    def checkedReturn(self, rawResponse):
        if rawResponse is None:
            return None

        return outgoing_manual(rawResponse)

    def getClassificationByUID(self, id):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type
        """
        selectedClass = super(OutgoingManualDAO, self).getClassificationByUID(id)
        return self.checkedReturn(selectedClass)

    def getAll(self):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type
        """
        cur = super(OutgoingManualDAO, self).getAll()

        results = []
        if cur is not None:
            for row in cur:
                outManualRow = outgoing_manual(row)
                results.append(outManualRow)

            cur.close() # dont forget to close the cursor
        
        return results

    def getClassification(self, id):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type
        """
        selectedClass = super(OutgoingManualDAO, self).getClassification(id)
        return self.checkedReturn(selectedClass)

    def updateClassificationByUID(self, id, updateClass):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type. We're also properly setting up the 
        initial model of stuff to update before passing it to super
        """

        updateModel = outgoing_manual(json=updateClass)
        selectedClass = super(OutgoingManualDAO, self).updateClassificationByUID(id, updateModel)
        return selectedClass # the above getClassification handle putting this into the proper object

    def getAllDistinct(self):
        return super(OutgoingManualDAO, self).getAllDistinct(self)
    
    def getAllDistinctPending(self):
        """
        Get images grouped by distinct targets pending submission (ei: submitted = false)
        """
        return super(OutgoingManualDAO, self).getAllDistinct(self, whereClause=" submitted = FALSE ")

    def newModelFromRow(self, row):
        """
        Kinda a reflective function for the classification dao. Pass self up
        to the super ClassificationDAO, and it calls this method to create the 
        proper model object in its response.

        Not uber elegant, only used by getAllDistinct atm.
        """
        return outgoing_manual(row)