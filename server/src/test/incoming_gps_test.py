import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.model.incoming_gps import incoming_gps


class TestGpsInsert(unittest.TestCase):
    def test(self):
        model = incoming_gps()
        model.time_stamp = 1547453775.2
        model.lat = 40.111
        model.lon = -111.222
        model.alt = 1234.5

        truncateTable('incoming_gps')
        dao = IncomingGpsDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        resultingId = dao.addGps(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

class TestGpsGetById(unittest.TestCase):
    def test(self):
        model = incoming_gps()
        model.time_stamp = 1547453775.2
        model.lat = 40.111
        model.lon = -111.222
        model.alt = 1234.5

        truncateTable('incoming_gps')
        dao = IncomingGpsDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        resultingId = dao.addGps(model)
        self.assertNotEqual(resultingId, -1)

        gottenMeas = dao.getGpsById(resultingId)
        self.assertIsNotNone(gottenMeas)

        self.assertEqual(gottenMeas.id, resultingId)
        self.assertAlmostEqual(gottenMeas.time_stamp, 1547453775.2)
        self.assertAlmostEqual(gottenMeas.lat, 40.111)
        self.assertAlmostEqual(gottenMeas.lon, -111.222)
        self.assertAlmostEqual(gottenMeas.alt, 1234.5)

class TestGpsGetByTs(unittest.TestCase):
    def test(self):
        truncateTable('incoming_gps')
        dao = IncomingGpsDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

        baseTs = 1547453775.2
        # test on empty table
        resultModel = dao.getGpsByClosestTS(baseTs)
        self.assertIsNone(resultModel)
        
        model = incoming_gps()
        model.time_stamp = baseTs
        model.lat = 40.111
        model.lon = -111.222
        model.alt = 1234.5
        resultingId1 = dao.addGps(model)
        self.assertNotEqual(resultingId1, -1)

        model.time_stamp = baseTs + 20000
        model.lat = 40.222
        model.lon = -111.333
        model.alt = 5678.9
        resultingId2 = dao.addGps(model)
        self.assertNotEqual(resultingId2, -1)

        resultModel = dao.getGpsByClosestTS(baseTs + 1000)
        self.assertIsNotNone(resultModel)
