import psycopg2 # postgres db connector
from config import config # to read config file

class BaseDAO(object):

    def __init__(self, configFilePath='../conf/config.ini'):
        """
        Startup the DAO. Attempts to connect to the postgresql database 
        using the settings specified in the confg.ini file
        """
        # get the connection settings for postgres:
        params = config(configFilePath, 'postgresql')

        print("Connecting to the database...")

        try:
            self.conn = psycopg2.connect(**params)
            self.conn.autocommit = True # automatically commit statements to table

            # run something simple to make sure we're really connected
            cur = self.conn.cursor()
            cur.execute('SELECT version()')
            db_version = cur.fetchone()
            print('Connected! Version :: {}'.format(db_version))
            cur.close()
        except (Exception, psycopg2.DatabaseError) as error:
            print("Something went wrong while trying to connect to the db!")
            print(error)
        finally:
            if self.conn is None:
                raise Exception("Something went wrong trying to connect!")



    @property
    def conn(self):
        return self._conn
    
    @conn.setter
    def conn(self, conn):
        self._conn = conn

    def basicInsert(self, stmt, values):
        """
        Performs a basic insert given the statement

        @type stmt: string
        @param stmt: The sql statement string to execute
        
        @type values: list
        @param values: Ordered list of values to place in the statement
        """

        if self.conn is None:
            print("wtf")

        cur = self.conn.cursor()
        cur.execute(stmt, values)
        ret = cur.fetchone()[0]
        cur.close()
        return ret