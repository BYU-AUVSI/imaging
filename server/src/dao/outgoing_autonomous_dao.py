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
        if selectedClass == -1:
            return selectedClass
        else:
            return outgoing_autonomous(selectedClass)
