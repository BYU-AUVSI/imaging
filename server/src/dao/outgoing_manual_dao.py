import psycopg2
from dao.classification_dao import ClassificationDAO
from dao.model.outgoing_manual import outgoing_manual

class OutgoingManualDAO(ClassificationDAO):
    """
    Outgoing_manual wrapper for the ClassificationDAO. Most of the core
    functionality here happens in the ClassificationDAO
    """

    def __init__(self, configFilePath):
        super(OutgoingManualDAO, self).__init__(configFilePath, 'outgoing_manual', 'crop_id')

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

    def updateClassification(self, id, updateClass):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type. We're also properly setting up the 
        initial model of stuff to update before passing it to super
        """

        updateModel = outgoing_manual(json=updateClass)
        selectedClass = super(OutgoingManualDAO, self).updateClassification(id, updateModel)
        return selectedClass # this is already the proper model object

    def updateClassificationByUID(self, id, updateClass):
        """
        See classification_dao docs. Here we're just making sure we cast the final object 
        to the proper outgoing classification model type. We're also properly setting up the 
        initial model of stuff to update before passing it to super
        """

        updateModel = outgoing_manual(json=updateClass)
        selectedClass = super(OutgoingManualDAO, self).updateClassificationByUID(id, updateModel)
        return selectedClass # this is already the proper model object

    def getAllDistinct(self):
        return super(OutgoingManualDAO, self).getAllDistinct(self)
    
    def getPendingTargets(self):
        """
        Get images grouped by distinct targets pending submission (ei: submitted = false)
        """
        return super(OutgoingManualDAO, self).getAllTargets(self, whereClause=" submitted = 'unsubmitted' ")

    def submitAllPendingTargets(self):
        return None

    def submitPendingTarget(self, target):
        """
        Submit the specified pending target to the judges.

        @return: an outgoing_manual object that can be used to submit the final classification
        """
        return super(OutgoingManualDAO, self).submitPendingTarget(self, target)

    def newModelFromRow(self, row):
        """
        Kinda a reflective function for the classification dao. Pass self up
        to the super ClassificationDAO, and it calls this method to create the 
        proper model object in its response.
        """
        return outgoing_manual(row)