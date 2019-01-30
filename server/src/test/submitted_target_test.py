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
