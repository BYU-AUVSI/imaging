import unittest
from client_rest import ImagingInterface
from client_rest import Classification
from test.test_helpers import resetAutonomousDb as resetDb

class TestAutonomousSubmitPendingGet(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface(isManual=False)

        # should get none if classification/pend list is empty:
        self.assertIsNone(rest.getPendingSubmissions())

        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)

        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped

        cropId = int(resp.headers['X-Crop-Id'])

        classToPost = Classification(cropId, 'standard', 'NE', 'circle', 'white', 'T', 'yellow')

        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = resp.headers['X-Class-Id']

        resp = rest.getPendingSubmissions()
        self.assertIsNotNone(resp)
        self.assertEqual(len(resp), 1)
        self.assertEqual(len(resp[0]), 1)
        self.assertEqual(resp[0][0].id, int(classId))
        self.assertEqual(resp[0][0].crop_id, int(cropId))

        # post a second crop and classification which goes to a different target:
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId2 = int(resp.headers['X-Crop-Id'])
        # since it's shape is different, this should be a different target
        classToPost = Classification(cropId2, 'standard', 'NE', 'square', 'orange', 'T', 'black')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId2 = int(resp.headers['X-Class-Id'])
        resp = rest.getPendingSubmissions()
        self.assertIsNotNone(resp)
        self.assertEqual(len(resp), 2)
        self.assertEqual(len(resp[0]), 1)
        self.assertEqual(len(resp[1]), 1)

        # post a third crop and classification which goes to one of our already existing targets
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId3 = int(resp.headers['X-Crop-Id'])
        # this should be a match with target2
        classToPost = Classification(cropId3, 'standard', 'SE', 'square', 'purple', 'T', 'black')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId3 = int(resp.headers['X-Class-Id'])
        self.assertNotEqual(classId2, classId3)

        resp = rest.getPendingSubmissions()
        self.assertIsNotNone(resp)
        self.assertEqual(len(resp), 2)

        if len(resp[0]) == 2:
            self.assertEqual(resp[0][1].type, 'standard')
            self.assertEqual(resp[0][1].shape, 'square')
            self.assertEqual(resp[0][0].alphanumeric, 'T')
            self.assertEqual(resp[1][0].id, int(classId))
            self.assertEqual(resp[1][0].crop_id, int(cropId))
        elif len(resp[1]) == 2:
            self.assertEqual(resp[0][0].id, int(classId))
            self.assertEqual(resp[0][0].crop_id, int(cropId))
            self.assertEqual(resp[1][1].type, 'standard')
            self.assertEqual(resp[1][1].shape, 'square')
            self.assertEqual(resp[1][0].alphanumeric, 'T')
        else:
            self.fail(msg="one of the targets should have two classifications")