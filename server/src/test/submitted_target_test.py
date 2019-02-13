import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.submitted_target_dao import SubmittedTargetDAO
from dao.model.submitted_target import submitted_target

class TestSubmittedTargetAConnection(unittest.TestCase):
    def test(self):
        dao = SubmittedTargetDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        self.assertIsNotNone(dao.conn)

class TestSubmittedTargetAdd(unittest.TestCase):
    def test(self):
        truncateTable('submitted_target')
        dao = SubmittedTargetDAO(defaultConfigPath())

        # none should fail to insert
        result = dao.upsertTarget(None)
        self.assertIsNotNone(result)
        self.assertEqual(result, -1)

        testIns = submitted_target()
        testIns.shape = 'square'
        testIns.type = 'standard'
        testIns.crop_path = '/another/totally/real/crop/path/ikr.jpg'
        testIns.background_color = 'white'
        testIns.alphanumeric = "T"
        testIns.alphanumeric_color = "orange"

        # should fail when we try and upsert an image that doesn't have a target or autonomous field
        self.assertEqual(dao.upsertTarget(testIns), -1)

        testIns.autonomous = False
        # should fail when we try and upsert an image that doesn't have a target field
        self.assertEqual(dao.upsertTarget(testIns), -1)

        testIns.target = 10
        self.assertNotEqual(dao.upsertTarget(testIns), -1)

class TestSubmittedTargetGet(unittest.TestCase):
    def test(self):
        truncateTable('submitted_target')
        dao = SubmittedTargetDAO(defaultConfigPath())

        testIns = submitted_target()
        testIns.shape = 'square'
        testIns.type = 'standard'
        testIns.crop_path = '/another/totally/real/crop/path/ikr.jpg'
        testIns.background_color = 'white'
        testIns.alphanumeric = "T"
        testIns.alphanumeric_color = "orange"
        testIns.autonomous = False
        testIns.target = 10

        self.assertNotEqual(dao.upsertTarget(testIns), -1)

        resultingModel = dao.getTarget(10, False)

        self.assertIsNotNone(resultingModel)
        self.assertEqual(resultingModel.target, 10)
        self.assertFalse(resultingModel.autonomous)
        self.assertEqual(resultingModel.type, 'standard')
        self.assertEqual(resultingModel.crop_path, '/another/totally/real/crop/path/ikr.jpg')
        self.assertEqual(resultingModel.shape, 'square')
        self.assertEqual(resultingModel.background_color, 'white')
        self.assertEqual(resultingModel.alphanumeric, 'T')
        self.assertEqual(resultingModel.alphanumeric_color, 'orange')
        self.assertEqual(resultingModel.submitted, 'pending')
        self.assertIsNone(resultingModel.latitude)
        self.assertIsNone(resultingModel.longitude)
        self.assertIsNone(resultingModel.orientation)

class TestSubmittedTargetGetAll(unittest.TestCase):
    def test(self):
        truncateTable('submitted_target')
        dao = SubmittedTargetDAO(defaultConfigPath())

        # empty table
        self.assertIsNone(dao.getAllTargets(False))
        self.assertIsNone(dao.getAllTargets(True))

        testIns = submitted_target()
        testIns.shape = 'square'
        testIns.type = 'standard'
        testIns.crop_path = '/another/totally/real/crop/path/ikr.jpg'
        testIns.background_color = 'white'
        testIns.alphanumeric = "T"
        testIns.alphanumeric_color = "orange"
        testIns.autonomous = False
        testIns.target = 10
        self.assertNotEqual(dao.upsertTarget(testIns), -1)

        testIns.shape = 'circle'
        testIns.target = 15
        self.assertNotEqual(dao.upsertTarget(testIns), -1)

        testIns.autonomous = True
        testIns.alphanumeric = "Y"
        testIns.alphanumeric_color = 'purple'
        self.assertNotEqual(dao.upsertTarget(testIns), -1)

        targetList = dao.getAllTargets(False)
        
        self.assertIsNotNone(targetList)
        self.assertEqual(len(targetList), 2)
        # spot check a couple values to confirm the thing was really added to list properly
        self.assertEqual(targetList[0].type, 'standard')
        self.assertEqual(targetList[1].type, 'standard')
        self.assertEqual(targetList[0].crop_path, '/another/totally/real/crop/path/ikr.jpg')
        self.assertEqual(targetList[1].crop_path, '/another/totally/real/crop/path/ikr.jpg')
        self.assertFalse(targetList[0].autonomous)
        self.assertFalse(targetList[1].autonomous)

        # get the one autonomous target
        targetList = dao.getAllTargets(True)
        self.assertIsNotNone(targetList)
        self.assertEqual(len(targetList), 1)

class TestSubmittedTargetRemove(unittest.TestCase):
    def test(self):
        truncateTable('submitted_target')
        dao = SubmittedTargetDAO(defaultConfigPath())

        self.assertFalse(dao.removeTarget(10, False))
        self.assertFalse(dao.removeTarget(10, True))

        testIns = submitted_target()
        testIns.shape = 'square'
        testIns.type = 'standard'
        testIns.crop_path = '/another/totally/real/crop/path/ikr.jpg'
        testIns.background_color = 'white'
        testIns.alphanumeric = "T"
        testIns.alphanumeric_color = "orange"
        testIns.autonomous = False
        testIns.target = 10
        self.assertNotEqual(dao.upsertTarget(testIns), -1)

        testIns.shape = 'circle'
        testIns.target = 15
        self.assertNotEqual(dao.upsertTarget(testIns), -1)

        self.assertFalse(dao.removeTarget(10, True))
        self.assertFalse(dao.removeTarget(12, False))
        self.assertTrue(dao.removeTarget(10, False))
        
        # confirm it got deleted
        self.assertIsNone(dao.getTarget(10, False))
        targetList = dao.getAllTargets(False)
        self.assertIsNotNone(targetList)
        self.assertEqual(len(targetList), 1)
        self.assertEqual(targetList[0].target, 15)
        self.assertEqual(targetList[0].shape, 'circle')

        self.assertTrue(dao.removeTarget(15, False))
        # confirm the table is now empty
        self.assertIsNone(dao.getAllTargets(False))
        self.assertIsNone(dao.getAllTargets(True))