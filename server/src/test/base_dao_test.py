import unittest
from dao.base_dao import BaseDAO
from config import defaultConfigPath

class TestGetConn(unittest.TestCase):
    def test(self):
        # basically just confirm that we're able
        # to connect
        dao = BaseDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        self.assertIsNotNone(dao.conn)

# basically all the other methods in BaseDao are tested by
# the other dao tests

class TestCloseConn(unittest.TestCase):
    def test(self):
        dao = BaseDAO(defaultConfigPath())

        self.assertIsNotNone(dao.conn)
        dao.close()
        self.assertIsNone(dao.conn)