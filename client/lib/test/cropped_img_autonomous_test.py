import unittest
from client_rest import ImagingInterface
from client_rest import CropInfo
from test.test_helpers import resetAutonomousDb

class TestAutonomousCroppedPost(unittest.TestCase):
    def test(self):
        resetAutonomousDb()

        rest = ImagingInterface(isManual=False)
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)

        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post

        cropId = int(resp.headers['X-Crop-Id'])
        self.assertIsNotNone(cropId)
        self.assertIsInstance(cropId, int)
        self.assertNotEqual(cropId, -1)

class TestAutonomousCroppedGetNext(unittest.TestCase):
    def test(self):
        resetAutonomousDb()
        
        rest = ImagingInterface(isManual=False)
        # table should be empty
        self.assertIsNone(rest.getNextCroppedImage())

        # push image into the table so that we can get it
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        cropId = int(resp.headers['X-Crop-Id'])

        # post a second image so we can confirm it gets the first posted one first
        resp = rest.postCroppedImage(ret[1], ret[0], [33, 33], [789, 789])
        cropId2 = int(resp.headers['X-Crop-Id'])

        self.assertNotEqual(cropId, cropId2)

        result = rest.getNextCroppedImage()
        self.assertIsNotNone(result)
        self.assertIsNotNone(result[0])
        self.assertEqual(result[1], cropId)

        result = rest.getNextCroppedImage()
        self.assertIsNotNone(result)
        self.assertIsNotNone(result[0])
        self.assertEqual(result[1], cropId2)

class TestAutonomousCroppedGetId(unittest.TestCase):
    def test(self):
        resetAutonomousDb()

        rest = ImagingInterface(isManual=False)
        # table should be empty
        self.assertIsNone(rest.getCroppedImage(100))

        # push image into the table so that we can get it
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        cropId = int(resp.headers['X-Crop-Id'])

        resp = rest.getCroppedImage(cropId)
        self.assertIsNotNone(resp)
        self.assertIsNotNone(resp[0])
        self.assertEqual(resp[1], cropId)

class TestAutonomousCroppedGetIdInfo(unittest.TestCase):
    def test(self):
        resetAutonomousDb()

        rest = ImagingInterface(isManual=False)
        # table should be empty
        self.assertIsNone(rest.getCroppedImageInfo(100))

        # push image into the table so that we can get it
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        cropId = int(resp.headers['X-Crop-Id'])
        # post a second image just for funnsies
        resp = rest.postCroppedImage(ret[1], ret[0], [33, 33], [789, 789])
        cropId2 = int(resp.headers['X-Crop-Id'])

        resp = rest.getCroppedImageInfo(cropId)
        self.assertIsNotNone(resp)
        # verify the returned data is what we put in
        self.assertEqual(resp.cropId, cropId) #crop_id
        self.assertEqual(resp.imgId, ret[1]) #image_id
        self.assertTrue(hasattr(resp, 'path'))
        self.assertTrue(hasattr(resp, 'time_stamp'))
        self.assertEqual(resp.tl[0], 22)
        self.assertEqual(resp.tl[1], 22)
        self.assertEqual(resp.br[0], 236)
        self.assertEqual(resp.br[1], 236)
        self.assertFalse(resp.isTapped)

class TestAutonomousCroppedGetAll(unittest.TestCase):
    def test(self):
        resetAutonomousDb()

        rest = ImagingInterface(isManual=False)
        # table should be empty
        self.assertIsNone(rest.getCroppedImageInfo(100))

        # push image into the table so that we can get it
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        cropId = int(resp.headers['X-Crop-Id'])
        # post a second image just for funnsies
        resp = rest.postCroppedImage(ret[1], ret[0], [33, 33], [789, 789])
        cropId2 = int(resp.headers['X-Crop-Id'])

        resp = rest.getAllCroppedInfo()
        self.assertIsNotNone(resp)
        # verify the returned data is what we put in
        self.assertEqual(len(resp), 2)

        modelA = resp[0]
        modelB = resp[1]
        if modelA.cropId == cropId:
            self.assertEqual(modelA.imgId, ret[1]) #image_id
            self.assertTrue(hasattr(modelA, 'path'))
            self.assertTrue(hasattr(modelA, 'time_stamp'))
            self.assertEqual(modelA.tl[0], 22)
            self.assertEqual(modelA.tl[1], 22)
            self.assertEqual(modelA.br[0], 236)
            self.assertEqual(modelA.br[1], 236)
            self.assertFalse(modelA.isTapped)

            # check model B
            self.assertEqual(modelB.imgId, ret[1]) #image_id
            self.assertTrue(hasattr(modelB, 'path'))
            self.assertTrue(hasattr(modelB, 'time_stamp'))
            self.assertEqual(modelB.tl[0], 33)
            self.assertEqual(modelB.tl[1], 33)
            self.assertEqual(modelB.br[0], 789)
            self.assertEqual(modelB.br[1], 789)
            self.assertFalse(modelB.isTapped)
        elif modelA.cropId == cropId2:
            self.assertEqual(modelA.imgId, ret[1]) #image_id
            self.assertTrue(hasattr(modelA, 'path'))
            self.assertTrue(hasattr(modelA, 'time_stamp'))
            self.assertEqual(modelA.tl[0], 33)
            self.assertEqual(modelA.tl[1], 33)
            self.assertEqual(modelA.br[0], 789)
            self.assertEqual(modelA.br[1], 789)
            self.assertFalse(modelA.isTapped)

            #check model b
            self.assertEqual(modelB.imgId, ret[1]) #image_id
            self.assertTrue(hasattr(modelB, 'path'))
            self.assertTrue(hasattr(modelB, 'time_stamp'))
            self.assertEqual(modelB.tl[0], 22)
            self.assertEqual(modelB.tl[1], 22)
            self.assertEqual(modelB.br[0], 236)
            self.assertEqual(modelB.br[1], 236)
            self.assertFalse(modelB.isTapped)
        else:
            self.fail()