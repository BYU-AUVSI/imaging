import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.model.manual_cropped import manual_cropped
from dao.manual_cropped_dao import ManualCroppedDAO

class TestManualCroppedConnection(unittest.TestCase):
    def test(self):
        dao = ManualCroppedDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

class TestCropImageAdd(unittest.TestCase):
    def test(self):
        """
        Currently the method that we're testing here isnt used by the api, but
        its good to test andways since it exists, and is pretty basic - meaning it
        will catch some simpler errors before we go to the complicated ones
        """
        model = manual_cropped()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'

        truncateTable('manual_cropped')
        dao = ManualCroppedDAO(defaultConfigPath())

        self.assertEqual(dao.addImage(None), -1)

        resultingId = dao.addImage(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

class TestCropImageUpsert(unittest.TestCase):
    def test(self):
        model = manual_cropped()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12, 34)"
        model.crop_coordinate_tl = "(56, 78)"

        truncateTable('manual_cropped')
        dao = ManualCroppedDAO(defaultConfigPath())

        self.assertEqual(dao.upsertCropped(None), -1)
        self.assertEqual(dao.upsertCropped(manual_cropped()), -1)

        resultingId = dao.upsertCropped(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

        model.crop_coordinate_br = "(56, 78)"
        model.id = resultingId
        resultingId2 = dao.upsertCropped(model)
        self.assertNotEqual(resultingId2, -1)
        self.assertEqual(resultingId, resultingId2)

class TestCropImageGet(unittest.TestCase):
    def test(self):
        model = manual_cropped()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12,34)"
        model.crop_coordinate_tl = "(56,78)"

        truncateTable('manual_cropped')
        dao = ManualCroppedDAO(defaultConfigPath())
        resultingId = dao.upsertCropped(model)

        self.assertNotEqual(resultingId, -1)
        resultingModel = dao.getImage(resultingId)

        self.assertIsNotNone(resultingModel)
        self.assertEqual(resultingModel.id, resultingId)
        self.assertEqual(resultingModel.image_id, model.image_id)
        self.assertEqual(resultingModel.cropped_path, model.cropped_path)
        self.assertEqual(resultingModel.crop_coordinate_br.__str__(), model.crop_coordinate_br)
        self.assertEqual(resultingModel.crop_coordinate_tl.__str__(), model.crop_coordinate_tl)
        self.assertFalse(resultingModel.tapped)

class TestCropImageGetNext(unittest.TestCase):
    def test(self):
        truncateTable('manual_cropped')
        dao = ManualCroppedDAO(defaultConfigPath())

        self.assertIsNone(dao.getNextImage())

        model = manual_cropped()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12,34)"
        model.crop_coordinate_tl = "(56,78)"
        resultingId = dao.upsertCropped(model)
        self.assertNotEqual(resultingId, -1)

        model.cropped_path = '/im/a/totally/real/cropped/path/2/i/swear.jpg'
        model.image_id = 456
        resultingId2 = dao.upsertCropped(model)
        self.assertNotEqual(resultingId2, -1)

        resultModel = dao.getNextImage()
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId)
        self.assertTrue(resultModel.tapped)
        self.assertEqual(resultModel.image_id, 123)
        self.assertEqual(resultModel.crop_coordinate_tl.__str__(), model.crop_coordinate_tl)
        self.assertEqual(resultModel.crop_coordinate_br.__str__(), model.crop_coordinate_br)

        resultModel = dao.getNextImage()
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.id, resultingId2)
        self.assertTrue(resultModel.tapped)
        self.assertEqual(resultModel.image_id, model.image_id)
        self.assertEqual(resultModel.cropped_path, model.cropped_path)
        self.assertEqual(resultModel.crop_coordinate_tl.__str__(), model.crop_coordinate_tl)
        self.assertEqual(resultModel.crop_coordinate_br.__str__(), model.crop_coordinate_br)

        self.assertIsNone(dao.getNextImage())

class TestCropImageGetAll(unittest.TestCase):
    def test(self):
        truncateTable('manual_cropped')
        dao = ManualCroppedDAO(defaultConfigPath())

        self.assertEqual(dao.getAll(), [])