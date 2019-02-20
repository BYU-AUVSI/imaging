from dao.base_dao import BaseDAO
from config import defaultConfigPath

def truncateTable(tableName):
    truncateSql = "TRUNCATE TABLE " + tableName

    dao = BaseDAO(defaultConfigPath())
    dao.executeStatements((truncateSql,))