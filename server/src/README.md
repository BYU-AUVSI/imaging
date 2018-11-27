# What's happening

There are two main scripts that are run here:

`server.py` Runs the entire REST server

`ros_ingest.py` Runs the ingester, which connects ROS to the database. It subscribes to appropriate ros_topics and then pushes their messages into a database using the appropriate DAO.

## Structure

`apis` Define all the REST api handlers for the various components of the server. We're using [Flask-RESTPlus](https://flask-restplus.readthedocs.io) as the framework for the REST api, since it's (relatively in terms of python frameworks) lightweight and very easy to use.

`dao` Contains all the database access objects and their corresponding model classes. We could have used SqlAlchemy, but we were going for something lean, hence just the base postgres python connector (psycopg2) was used.

`test` Contains some simple test code. Not used in production