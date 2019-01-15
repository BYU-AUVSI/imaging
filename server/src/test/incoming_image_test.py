import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.model.incoming_image import incoming_image
from dao.incoming_image_dao import IncomingImageDAO

class TestImageInsert(unittest.TestCase):
    def test(self):
        model = incoming_image()
        model.time_stamp = 1547453775.2
        model.focal_length = 16.0
        model.image_path = '/im/a/totally/real/path/i/swear.jpg'
        model.manual_tap = False
        model.autonomous_tap = False

        truncateTable('incoming_image')
        dao = IncomingImageDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

        self.assertEqual(dao.addImage(None), -1)

        resultingId = dao.addImage(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

class TestImageGet(unittest.TestCase):
    def test(self):
        model = incoming_image()
        model.time_stamp = 1547453775.2
        model.focal_length = 16.0
        model.image_path = '/im/a/totally/real/path/i/swear.jpg'
        model.manual_tap = False
        model.autonomous_tap = False

        truncateTable('incoming_image')
        dao = IncomingImageDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

        # test with empty table

        self.assertIsNone(dao.getImage(1))

        resultingId = dao.addImage(model)
        self.assertNotEqual(resultingId, -1)

        resultingModel = dao.getImage(resultingId)
        
        self.assertIsNotNone(resultingModel)
        self.assertAlmostEqual(resultingModel.time_stamp, model.time_stamp)
        self.assertEqual(resultingModel.focal_length, model.focal_length)
        self.assertEqual(resultingModel.image_path, model.image_path)
        self.assertEqual(resultingModel.manual_tap, model.manual_tap)
        self.assertEqual(resultingModel.autonomous_tap, model.autonomous_tap)

class TestImageGetNext(unittest.TestCase):
    def test(self):
        truncateTable('incoming_image')
        dao = IncomingImageDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

        # test with empty table
        self.assertIsNone(dao.getNextImage(True))
        self.assertIsNone(dao.getNextImage(False))

        model = incoming_image()
        model.time_stamp = 1547453775.2
        model.focal_length = 16.0
        model.image_path = '/im/a/totally/real/path/i/swear.jpg'
        model.manual_tap = False
        model.autonomous_tap = False
        resultingId = dao.addImage(model)
        self.assertNotEqual(resultingId, -1)

        # identical timestamps should make no difference
        model.focal_length = 16.0
        model.image_path = '/im/a/totally/real/path/2/i/swear.jpg'
        resultingId2 = dao.addImage(model)
        self.assertNotEqual(resultingId2, -1)


        resultModel = dao.getNextImage(True)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.image_id, resultingId)
        self.assertTrue(resultModel.manual_tap)
        self.assertFalse(resultModel.autonomous_tap)

        resultModel = dao.getNextImage(True)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.image_id, resultingId2)
        self.assertTrue(resultModel.manual_tap)
        self.assertFalse(resultModel.autonomous_tap)

        resultModel = dao.getNextImage(False)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.image_id, resultingId)
        self.assertTrue(resultModel.manual_tap)
        self.assertTrue(resultModel.autonomous_tap)

        resultModel = dao.getNextImage(True)
        self.assertIsNone(resultModel)

        resultModel = dao.getNextImage(False)
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.image_id, resultingId2)
        self.assertTrue(resultModel.manual_tap)
        self.assertTrue(resultModel.autonomous_tap)

        resultModel = dao.getNextImage(False)
        self.assertIsNone(resultModel)
