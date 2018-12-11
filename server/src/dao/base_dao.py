import psycopg2 # postgres db connector
from config import config # to read config file

class BaseDAO(object):
    """
    DAO with basic methods. All other DAO's are child classes of BaseDAO.
    Initializes and contains a postgres connection object when created.
    """

    def __init__(self, configFilePath='../conf/config.ini'):
        """
        Startup the DAO. Attempts to connect to the postgresql database 
        using the settings specified in the confg.ini file
        """
        # get the connection settings for postgres:
        params = config(configFilePath, 'postgresql')

        try:
            self.conn = psycopg2.connect(**params)
            self.conn.autocommit = True # automatically commit statements to table
        except (Exception, psycopg2.DatabaseError) as error:
            print("Something went wrong while trying to connect to the db!")
            print(error)
        finally:
            if self.conn is None:
                raise Exception("Something went wrong trying to connect!")

    def close(self):
        """
        Safely close the DAO's connection. It is higly recommended you call this method
        before finishing with a dao.
        """
        self.conn.close()

    @property
    def conn(self):
        """
        Current postgres connection object
        """
        return self._conn
    
    @conn.setter
    def conn(self, conn):
        self._conn = conn

    def getResultingId(self, stmt, values):
        """
        Get the first id returned from a statement.
        Basically this assumes you have a 'RETURNING id;' at the end
        of the query you are executing (insert or update)

        @type stmt: string
        @param stmt: The sql statement string to execute
        
        @type values: list
        @param values: Ordered list of values to place in the statement
        """
        try:
            cur = self.conn.cursor()
            cur.execute(stmt, values)
            ret = cur.fetchone()
            if ret is not None:
                ret = ret[0]
            else:
                ret = -1
            cur.close()
            return ret
        except (Exception) as error:
            print(error)
            return -1


    def executeStatements(self, stmts):
        """
        Tries to execute all SQL statements in the stmts list.
        These will be performed in a single transaction.
        Returns nothing, so useless if you're trying to execute a series of fetches

        @type stmts: [string]
        @param stmts: List of sql statements to execute
        """
        cur = self.conn.cursor()
        # we're going to do all statements in a single transaction
        self.conn.autocommit = False 
        for stmt in stmts:
            cur.execute(stmt)

        self.conn.commit()
        cur.close()
        self.conn.autocommit = True

    def basicTopSelect(self, stmt, values):
        """
        Gets the first (top) row of the given select statement.

        @type stmt: string
        @param stmt: Sql statement string to run

        @type values: [object]
        @param values: List of objects (generally int, float and string), to safely place in the 
                       sql statement.

        @rtype: [string]
        @return: The first row of the select stmt's result. If the statement fails or does not retrieve
                 any records, None is returned.
        """
        try:
            cur = self.conn.cursor()
            cur.execute(stmt, values)
            ret = cur.fetchone()
            cur.close()
            return ret
        except (Exception) as error:
            print(error)
            return None

