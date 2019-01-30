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

        selectedTarget = super(SubmittedTargetDAO, self).basicTopSelect(getTarget, (id, autonomous))
        if selectedTarget is not None:
            return submitted_target(selectedTarget)
        return None

    def getAllTargets(self, autonomous):
        getTarget = """SELECT * FROM submitted_target 
            WHERE autonomous = %s LIMIT 1;"""

        cur = self.conn.cursor()
        if cur is None:
            cur.close()
            return None

        cur.execute(getTarget, (autonomous,))

        targetList = []
        rawRecords = cur.fetchall()

        for record in rawRecords:
            targetList.append(record)

        return targetList
        
    def removeTarget(self, target, autonomous):
        removeSql = "DELETE FROM submitted_target WHERE target = %s and autonomous = %s;"

        rcount = super(SubmittedTargetDAO, self).getNumAffectedRows(removeSql, (target, autonomous))

        return rcount > 0