# What's happening

There are two main scripts that are run here:

`server.py` Runs the entire REST server

`ros_ingest.py` Runs the ingester, which connects ROS to the database. It subscribes to appropriate ros_topics and then pushes their messages into a database using the appropriate DAO.

## Other useful stuff

`test.py` Is a supplemental script used to run unit tests. Should be run when changes are made to dao modules, or from a continuous integration platform

`generate_api_tests.py` Generates the `postman_import.json` file which can be imported as a collection into [Postman](https://www.getpostman.com). The collection will have all the possible endpoints and should make it easy for you to plug in values and test them out. The root page of the server (http://localhost:5000) uses swagger which also supports some quick-and-dirty testing.

## Structure

`apis` Define all the REST api handlers for the various components of the server. We're using [Flask-RESTPlus](https://flask-restplus.readthedocs.io) as the framework for the REST api, since it's (relatively in terms of python frameworks) lightweight and very easy to use.

`dao` Contains all the database access objects and their corresponding model classes. We could have used SqlAlchemy, but we were going for something lean, hence just the base postgres python connector (psycopg2) was used.

`test` Contains the test code run by test.py. None of this code is used in a production environment