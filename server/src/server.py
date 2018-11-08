import psycopg2	# postgresql database connector
from config import config # reads our configuration file

def connect():
    """
    Attempts to connect to the postgresql database located in the place
    specified in the config.ini file
    """
    conn = None
    try:
         # read connection parameters
        params = config()
        print("settings:: {}".format(params))
        # connect to the PostgreSQL server
        print('Connecting to the PostgreSQL database...')
        conn = psycopg2.connect(**params)
 
        # run something simple to make sure we really connected
        cur = conn.cursor()
        cur.execute('SELECT version()')
 
        # display the PostgreSQL database server version
        db_version = cur.fetchone()
        print('Connected! Version :: {}'.format(db_version))
    
        # close cursor
        cur.close()
    except (Exception, psycopg2.DatabaseError) as error:
        print(error)
    finally:
        if conn is not None:
            return conn
        else:
            raise Exception("Something went wrong trying to connect!")

if __name__ == '__main__':
    conn = connect()