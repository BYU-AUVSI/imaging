import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.util_dao import UtilDAO
from dao.model.manual_cropped import manual_cropped
from dao.model.incoming_image import incoming_image
from dao.model.outgoing_manual import outgoing_manual
from dao.incoming_image_dao import IncomingImageDAO
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.manual_cropped_dao import ManualCroppedDAO
from dao.util_dao import UtilDAO
from dao.model.outgoing_autonomous import outgoing_autonomous
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO

class TestResetManualDB(unittest.TestCase):
    def test(self):
        truncateTable('manual_cropped')
        truncateTable('incoming_image')
        truncateTable('outgoing_manual')
        dao = OutgoingManualDAO(defaultConfigPath())

        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        dao = ManualCroppedDAO(defaultConfigPath())
        model = manual_cropped()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12, 34)"
        model.crop_coordinate_tl = "(56, 78)"
        self.assertNotEqual(dao.addImage(model), -1)

        dao = IncomingImageDAO(defaultConfigPath())
        model = incoming_image()
        model.time_stamp = 1547453775.2
        model.focal_length = 16.0
        model.image_path = '/im/a/totally/real/path/i/swear.jpg'
        model.manual_tap = True
        model.autonomous_tap = False
        resultingId = dao.addImage(model)
        self.assertNotEqual(resultingId, -1)

        util = UtilDAO(defaultConfigPath())
        util.resetManualDB()

        resultingModel = dao.getImage(resultingId)
        self.assertIsNotNone(resultingModel)
        self.assertFalse(resultingModel.manual_tap)
        self.assertEqual(resultingModel.image_path, model.image_path)
        self.assertEqual(resultingModel.focal_length, model.focal_length)
        
        dao = ManualCroppedDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

        dao = OutgoingManualDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

class TestResetAutonomousDB(unittest.TestCase):
    def test(self):
        truncateTable('incoming_image')
        truncateTable('outgoing_autonomous')
        dao = OutgoingAutonomousDAO(defaultConfigPath())

        testIns = outgoing_autonomous()
        testIns.image_id = 123
        testIns.crop_path = '/am/i/even/a/real/path/idk/anymore.jpg'
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        dao = IncomingImageDAO(defaultConfigPath())
        model = incoming_image()
        model.time_stamp = 1547453775.2
        model.focal_length = 16.0
        model.image_path = '/im/a/totally/real/path/i/swear.jpg'
        model.manual_tap = True
        model.autonomous_tap = True
        resultingId = dao.addImage(model)
        self.assertNotEqual(resultingId, -1)

        util = UtilDAO(defaultConfigPath())
        util.resetAutonomousDB()

        resultingModel = dao.getImage(resultingId)
        self.assertIsNotNone(resultingModel)
        self.assertFalse(resultingModel.autonomous_tap)
        self.assertTrue(resultingModel.manual_tap)
        self.assertEqual(resultingModel.image_path, model.image_path)
        self.assertEqual(resultingModel.focal_length, model.focal_length)

        dao = OutgoingAutonomousDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)