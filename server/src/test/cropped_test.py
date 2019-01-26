import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.model.cropped_manual import cropped_manual
from dao.cropped_manual_dao import CroppedManualDAO

class TestManualCroppedConnection(unittest.TestCase):
    def test(self):
        dao = CroppedManualDAO(defaultConfigPath())
        self.assertIsNotNone(dao)

class TestCropImageAdd(unittest.TestCase):
    def test(self):
        """
        Currently the method that we're testing here isnt used by the api, but
        its good to test andways since it exists, and is pretty basic - meaning it
        will catch some simpler errors before we go to the complicated ones
        """
        model = cropped_manual()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'

        truncateTable('cropped_manual')
        dao = CroppedManualDAO(defaultConfigPath())

        self.assertEqual(dao.addImage(None), -1)

        resultingId = dao.addImage(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

class TestCropImageUpsert(unittest.TestCase):
    def test(self):
        model = cropped_manual()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12, 34)"
        model.crop_coordinate_tl = "(56, 78)"

        truncateTable('cropped_manual')
        dao = CroppedManualDAO(defaultConfigPath())

        self.assertEqual(dao.upsertCropped(None), -1)
        self.assertEqual(dao.upsertCropped(cropped_manual()), -1)

        resultingId = dao.upsertCropped(model)
        self.assertIsNotNone(resultingId)
        self.assertNotEqual(resultingId, -1)

        model.crop_coordinate_br = "(56, 78)"
        model.crop_id = resultingId
        resultingId2 = dao.upsertCropped(model)
        self.assertNotEqual(resultingId2, -1)
        self.assertEqual(resultingId, resultingId2)

class TestCropImageGet(unittest.TestCase):
    def test(self):
        model = cropped_manual()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12,34)"
        model.crop_coordinate_tl = "(56,78)"

        truncateTable('cropped_manual')
        dao = CroppedManualDAO(defaultConfigPath())
        resultingId = dao.upsertCropped(model)

        self.assertNotEqual(resultingId, -1)
        resultingModel = dao.getImage(resultingId)

        self.assertIsNotNone(resultingModel)
        self.assertEqual(resultingModel.crop_id, resultingId)
        self.assertEqual(resultingModel.image_id, model.image_id)
        self.assertEqual(resultingModel.cropped_path, model.cropped_path)
        self.assertEqual(resultingModel.crop_coordinate_br.__str__(), model.crop_coordinate_br)
        self.assertEqual(resultingModel.crop_coordinate_tl.__str__(), model.crop_coordinate_tl)
        self.assertFalse(resultingModel.tapped)

class TestCropImageGetNext(unittest.TestCase):
    def test(self):
        truncateTable('cropped_manual')
        dao = CroppedManualDAO(defaultConfigPath())

        self.assertIsNone(dao.getNextImage())

        model = cropped_manual()
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
        self.assertEqual(resultModel.crop_id, resultingId)
        self.assertTrue(resultModel.tapped)
        self.assertEqual(resultModel.image_id, 123)
        self.assertEqual(resultModel.crop_coordinate_tl.__str__(), model.crop_coordinate_tl)
        self.assertEqual(resultModel.crop_coordinate_br.__str__(), model.crop_coordinate_br)

        resultModel = dao.getNextImage()
        self.assertIsNotNone(resultModel)
        self.assertEqual(resultModel.crop_id, resultingId2)
        self.assertTrue(resultModel.tapped)
        self.assertEqual(resultModel.image_id, model.image_id)
        self.assertEqual(resultModel.cropped_path, model.cropped_path)
        self.assertEqual(resultModel.crop_coordinate_tl.__str__(), model.crop_coordinate_tl)
        self.assertEqual(resultModel.crop_coordinate_br.__str__(), model.crop_coordinate_br)

        self.assertIsNone(dao.getNextImage())

class TestCropImageGetAll(unittest.TestCase):
    def test(self):
        truncateTable('cropped_manual')
        dao = CroppedManualDAO(defaultConfigPath())

        self.assertEqual(dao.getAll(), [])

        model = cropped_manual()
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

        result = dao.getAll()

        self.assertEqual(len(result), 2)

class TestCropImageUpdate(unittest.TestCase):
    def test(self):
        truncateTable("cropped_manual")
        dao = CroppedManualDAO(defaultConfigPath())


        model = cropped_manual()
        model.image_id   = 123
        model.time_stamp = 1547453775.2
        model.cropped_path = '/im/a/totally/real/cropped/path/i/swear.jpg'
        model.crop_coordinate_br = "(12,34)"
        model.crop_coordinate_tl = "(56,78)"
        resultingId = dao.upsertCropped(model)
        self.assertNotEqual(resultingId, -1)

        updateContent = {
            "time_stamp": model.time_stamp + 1000,
            "image_id": 456
        }

        # confirm update to bad id does nothing
        self.assertIsNone(dao.updateImage(resultingId + 20, updateContent))
        
        resultingModel = dao.getImage(resultingId)
        # confirm that nothing was changed in our one row:
        self.assertIsNotNone(resultingModel)
        self.assertEqual(resultingId, resultingModel.crop_id)
        self.assertEqual(model.time_stamp, resultingModel.time_stamp)
        self.assertEqual(model.image_id, resultingModel.image_id)
        self.assertEqual(model.cropped_path, resultingModel.cropped_path)
        self.assertEqual(model.crop_coordinate_br.__str__(), resultingModel.crop_coordinate_br.__str__())
        self.assertEqual(model.crop_coordinate_tl.__str__(), resultingModel.crop_coordinate_tl.__str__())
        self.assertFalse(resultingModel.tapped)

        # do a legit update now:
        resultingModel = dao.updateImage(resultingId, updateContent)
        self.assertIsNotNone(resultingModel)
        self.assertEqual(resultingId, resultingModel.crop_id)
        self.assertEqual(resultingModel.image_id, 456)
        self.assertEqual(model.time_stamp + 1000, resultingModel.time_stamp)
        self.assertEqual(model.cropped_path, resultingModel.cropped_path)
        self.assertEqual(model.crop_coordinate_br.__str__(), resultingModel.crop_coordinate_br.__str__())
        self.assertEqual(model.crop_coordinate_tl.__str__(), resultingModel.crop_coordinate_tl.__str__())
        self.assertFalse(resultingModel.tapped)
