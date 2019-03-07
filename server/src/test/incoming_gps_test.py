import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.incoming_gps_dao import IncomingGpsDAO
from dao.model.incoming_gps import incoming_gps

class TestIncomingGpsConnection(unittest.TestCase):
    def test(self):
        dao = IncomingGpsDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        self.assertIsNotNone(dao.conn)
        
class TestGpsInsert(unittest.TestCase):
    def test(self):
        model = incoming_gps()
        model.time_stamp = 1547453775.2
        model.lat = 40.111
        model.lon = -111.222
        model.alt = 1234.5

        truncateTable('incoming_gps')
        dao = IncomingGpsDAO(defaultConfigPath())

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
        # Note: the closest TS function doesnt work by absolute closest,
        # but instead for ease, speed and readability, just checks <= 
        # These tests reflect this type of functionality and would probably have
        # to be redone if true closest TS was ever implemented

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

        # as explained above, this should return None
        resultModel = dao.getGpsByClosestTS(baseTs - 10000)
        self.assertIsNone(resultModel)

        resultModel = dao.getGpsByClosestTS(baseTs + 1000)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId1)

        # while this is absolutely closer to id2, we should
        # still get id1 for the spec reasons described at the 
        # top of this method
        resultModel = dao.getGpsByClosestTS(baseTs + 15000)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId1)

        # test when time is exactly equal
        resultModel = dao.getGpsByClosestTS(baseTs + 20000)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId2)

class TestGpsGetAll(unittest.TestCase):
    def test(self):
        truncateTable("incoming_gps")
        dao = IncomingGpsDAO(defaultConfigPath())

        # empty table
        self.assertIsNone(dao.getAll())

        # insert a couple rows
        model = incoming_gps()
        baseTs = 1547453775.2
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

        results = dao.getAll()
        self.assertIsNotNone(results) 
        self.assertEqual(len(results), 2)

        if results[0].id == resultingId1:
            self.assertAlmostEqual(results[0].lat, 40.111)
            self.assertAlmostEqual(results[0].lon, -111.222)
            self.assertAlmostEqual(results[0].alt, 1234.5)

            self.assertAlmostEqual(results[1].lat, 40.222)
            self.assertAlmostEqual(results[1].lon, -111.333)
            self.assertAlmostEqual(results[1].alt, 5678.9)
        
        elif results[0].id == resultingId2:
            self.assertAlmostEqual(results[1].lat, 40.111)
            self.assertAlmostEqual(results[1].lon, -111.222)
            self.assertAlmostEqual(results[1].alt, 1234.5)

            self.assertAlmostEqual(results[0].lat, 40.222)
            self.assertAlmostEqual(results[0].lon, -111.333)
            self.assertAlmostEqual(results[0].alt, 5678.9)
        
        else:
            self.fail("dont recognize one of the ids returned by gps.getAll")