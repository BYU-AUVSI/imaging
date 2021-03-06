import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.util_dao import UtilDAO
from dao.model.cropped_manual import cropped_manual
from dao.model.cropped_autonomous import cropped_autonomous
from dao.model.incoming_image import incoming_image
from dao.model.outgoing_manual import outgoing_manual
from dao.incoming_image_dao import IncomingImageDAO
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.cropped_manual_dao import CroppedManualDAO
from dao.cropped_autonomous_dao import CroppedAutonomousDAO
from dao.util_dao import UtilDAO
from dao.model.outgoing_autonomous import outgoing_autonomous
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO

class TestResetManualDB(unittest.TestCase):
    def test(self):
        truncateTable('cropped_manual')
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

        dao = CroppedManualDAO(defaultConfigPath())
        model = cropped_manual()
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
        
        dao = CroppedManualDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

        dao = OutgoingManualDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

class TestResetAutonomousDB(unittest.TestCase):
    def test(self):
        truncateTable('incoming_image')
        truncateTable('cropped_autonomous')
        truncateTable('outgoing_autonomous')
        dao = OutgoingAutonomousDAO(defaultConfigPath())

        testIns = outgoing_autonomous()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        dao = CroppedAutonomousDAO(defaultConfigPath())
        model = cropped_autonomous()
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

        dao = CroppedAutonomousDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

        dao = OutgoingAutonomousDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

class TestResetAll(unittest.TestCase):
    def test(self):
        """
        basically just a combo of the last two tests
        """
        truncateTable('incoming_image')
        truncateTable('cropped_autonomous')
        truncateTable('outgoing_autonomous')
        truncateTable('cropped_manual')
        truncateTable('outgoing_manual')

        dao = OutgoingManualDAO(defaultConfigPath())
        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        dao = CroppedManualDAO(defaultConfigPath())
        model = cropped_manual()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12, 34)"
        model.crop_coordinate_tl = "(56, 78)"
        self.assertNotEqual(dao.addImage(model), -1)

        dao = OutgoingAutonomousDAO(defaultConfigPath())
        testIns = outgoing_autonomous()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        dao = CroppedAutonomousDAO(defaultConfigPath())
        model = cropped_autonomous()
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
        model.autonomous_tap = True
        resultingId = dao.addImage(model)
        self.assertNotEqual(resultingId, -1)

        util = UtilDAO(defaultConfigPath())
        util.resetAll()

        self.assertEqual(len(dao.getAll()), 0) # asset incoming image empty
        
        dao = CroppedManualDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

        dao = OutgoingManualDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

        dao = CroppedAutonomousDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)

        dao = OutgoingAutonomousDAO(defaultConfigPath())
        self.assertEqual(len(dao.getAll()), 0)