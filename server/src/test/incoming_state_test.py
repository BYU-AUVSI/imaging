import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.incoming_state_dao import IncomingStateDAO
from dao.model.incoming_state import incoming_state

class TestIncomingStateConnection(unittest.TestCase):
    def test(self):
        dao = IncomingStateDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        self.assertIsNotNone(dao.conn)

class TestStateInsert(unittest.TestCase):
    def test(self):
        model = incoming_state()
        model.time_stamp = 1547453775.2
        model.roll = 40.111
        model.pitch = -111.222
        model.yaw = 12.3

        truncateTable('incoming_state')
        dao = IncomingStateDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        resultingId = dao.addState(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

class TestStateGetById(unittest.TestCase):
    def test(self):
        model = incoming_state()
        model.time_stamp = 1547453775.2
        model.roll = 40.111
        model.pitch = -111.222
        model.yaw = 12.3

        truncateTable('incoming_state')
        dao = IncomingStateDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        resultingId = dao.addState(model)
        self.assertNotEqual(resultingId, -1)

        gottenMeas = dao.getStateById(resultingId)
        self.assertIsNotNone(gottenMeas)

        self.assertEqual(gottenMeas.id, resultingId)
        self.assertAlmostEqual(gottenMeas.time_stamp, 1547453775.2)
        self.assertAlmostEqual(gottenMeas.roll, 40.111)
        self.assertAlmostEqual(gottenMeas.pitch, -111.222)
        self.assertAlmostEqual(gottenMeas.yaw, 12.3)

class TestStateGetByTs(unittest.TestCase):
    def test(self):
        # Note: the closest TS function doesnt work by absolute closest,
        # but instead for ease, speed and readability, just checks <= 
        # These tests reflect this type of functionality and would probably have
        # to be redone if true closest TS was ever implemented

        truncateTable('incoming_state')
        dao = IncomingStateDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

        baseTs = 1547453775.2
        # test on empty table
        resultModel = dao.getStateByClosestTS(baseTs)
        self.assertIsNone(resultModel)
        
        model = incoming_state()
        model.time_stamp = baseTs
        model.roll = 40.111
        model.pitch = -111.222
        model.yaw = 12.3
        resultingId1 = dao.addState(model)
        self.assertNotEqual(resultingId1, -1)

        model.time_stamp = baseTs + 20000
        model.roll = 40.222
        model.pitch = -111.333
        model.yaw = 567.8
        resultingId2 = dao.addState(model)
        self.assertNotEqual(resultingId2, -1)

        # as explained above, this should return None
        resultModel = dao.getStateByClosestTS(baseTs - 10000)
        self.assertIsNone(resultModel)

        resultModel = dao.getStateByClosestTS(baseTs + 1000)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId1)

        # while this is absolutely closer to id2, we should
        # still get id1 for the spec reasons described at the 
        # top of this method
        resultModel = dao.getStateByClosestTS(baseTs + 15000)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId1)

        # test when time is exactly equal
        resultModel = dao.getStateByClosestTS(baseTs + 20000)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId2)

