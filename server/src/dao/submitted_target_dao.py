import psycopg2
from dao.base_dao import BaseDAO
from dao.model.submitted_target import submitted_target

class SubmittedTargetDAO(BaseDAO):

    def __init__(self, configFilePath):
        super(SubmittedTargetDAO, self).__init__(configFilePath)

    def upsertTarget(self, targetModel):

        if targetModel is None:
            return -1

        insertCls = "INSERT INTO submitted_target "
        updateCls = "UPDATE SET "

        insertValues = []
        insertClmnNames = '('
        insertClmnValues = ' VALUES('
        for clmn, value in targetModel.toDict().items():
            if value is not None:
                insertClmnNames += clmn + ', '
                insertClmnValues += '%s, '
                updateCls += clmn + '= %s, '
                insertValues.append(value.__str__())

        # if there were no values to insert...
        if not insertValues:
            return -1
        else: 
            insertClmnNames = insertClmnNames[:-2] + ')' # remove last comma/space
            insertClmnValues = insertClmnValues[:-2] + ') ON CONFLICT (target, autonomous) DO '
            updateCls = updateCls[:-2] + 'RETURNING target;'

        insertCls += insertClmnNames + insertClmnValues + updateCls
        id = super(SubmittedTargetDAO, self).getResultingId(insertCls, insertValues + insertValues)
        return id

    def getTarget(self, target, autonomous):
        getTarget = """SELECT * FROM submitted_target 
            WHERE target = %s and autonomous = %s LIMIT 1;"""

        selectedTarget = super(SubmittedTargetDAO, self).basicTopSelect(getTarget, (target, autonomous))
        if selectedTarget is not None:
            return submitted_target(sqlRow=selectedTarget)
        return None

    def getAllTargets(self, autonomous):
        getTarget = """SELECT * FROM submitted_target 
            WHERE autonomous = %s;"""

        return super(SubmittedTargetDAO, self).getResultsAsModelList(getTarget, (autonomous,))

    def getAllPendingTargets(self, autonomous):
        getTarget = """SELECT * FROM submitted_target 
            WHERE autonomous = %s AND submitted = 'pending';"""

        return super(SubmittedTargetDAO, self).getResultsAsModelList(getTarget, (autonomous,))

    def removeTarget(self, target, autonomous):
        removeSql = "DELETE FROM submitted_target WHERE target = %s and autonomous = %s;"

        rcount = super(SubmittedTargetDAO, self).getNumAffectedRows(removeSql, (target, autonomous))

        return rcount > 0

    def newModelFromRow(self, row, json=None):
        return submitted_target(sqlRow=row)