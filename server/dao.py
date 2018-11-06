import psycopg2 # postgres db connector
from config import config # to read config file

class DAO:

    conn_ = None # the db connection object. Use this for queries

    def __init__(self):
        """
        Startup the DAO. Attempts to connect to the postgresql database 
        using the settings specified in the confg.ini file
        """
        # get the connection settings for postgres:
        params = config('conf/config.ini', 'postgresql')

        print("Connecting to the database...")

        try:
            self.conn_ = psycopg2.connect(**params)

            # run something simple to make sure we're really connected
            cur = self.conn_.cursor()
            cur.execute('SELECT version()')
            db_version = cur.fetchone()
            print('Connected! Version :: {}'.format(db_version))
            cur.close()
        except (Exception, psycopg2.DatabaseError) as error:
            print("Something went wrong while trying to connect to the db!")
            print(error)
        finally:
            if self.conn_ is None:
                raise Exception("Something went wrong trying to connect!")
