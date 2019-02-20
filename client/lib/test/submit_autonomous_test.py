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
        self.assertEqual(resp[0][0].class_id, int(classId))
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
            self.assertEqual(resp[1][0].class_id, int(classId))
            self.assertEqual(resp[1][0].crop_id, int(cropId))
        elif len(resp[1]) == 2:
            self.assertEqual(resp[0][0].class_id, int(classId))
            self.assertEqual(resp[0][0].crop_id, int(cropId))
            self.assertEqual(resp[1][1].type, 'standard')
            self.assertEqual(resp[1][1].shape, 'square')
            self.assertEqual(resp[1][0].alphanumeric, 'T')
        else:
            self.fail(msg="one of the targets should have two classifications")

class TestAutonomousSubmitTargetId(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface(isManual=False)

        # should get none when we try and post on empty table
        self.assertIsNone(rest.postSubmitTargetById(100))

        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId, 'standard', 'NE', 'circle', 'white', 'T', 'yellow')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = int(resp.headers['X-Class-Id'])
        modelResult = rest.getClassById(classId)
        self.assertIsNotNone(modelResult)
        targetId = modelResult.target

        # trying to submit an id that doesn't exist should also fail
        self.assertIsNone(rest.postSubmitTargetById(targetId + 20))

        resp = rest.postSubmitTargetById(targetId)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200)

        # confirm that the classification's status has changed to submitted
        modelResult = rest.getClassById(classId)
        self.assertIsNotNone(modelResult)
        self.assertIsNotNone(modelResult.target)
        self.assertEqual(modelResult.submitted, 'submitted')

        # trying to submit the same target again should fail:
        self.assertIsNone(rest.postSubmitTargetById(targetId))

class TestAutonomousSubmitGetTargetId(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface(isManual=False)

        # empty table
        self.assertIsNone(rest.getSubmittedTargetById(100))

        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId, 'standard', 'NE', 'circle', 'white', 'T', 'yellow')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = int(resp.headers['X-Class-Id'])
        modelResult = rest.getClassById(classId)
        self.assertIsNotNone(modelResult)
        targetId = modelResult.target

        # should still be none since the target is unsubmitted
        self.assertIsNone(rest.getSubmittedTargetById(targetId))

        # submit the target
        resp = rest.postSubmitTargetById(targetId)
        self.assertIsNotNone(resp)

        # now lets see if the getter works:
        resp = rest.getSubmittedTargetById(targetId)
        self.assertIsNotNone(resp)
        self.assertIsInstance(resp, Classification)
        self.assertEqual(resp.target, targetId)
        self.assertEqual(resp.crop_id, cropId)
        self.assertEqual(resp.type, 'standard')
        self.assertEqual(resp.shape, 'circle')
        self.assertEqual(resp.orientation, 'NE')
        self.assertEqual(resp.background_color, 'white')
        self.assertEqual(resp.alphanumeric, 'T')
        self.assertEqual(resp.alphanumeric_color, 'yellow')
        self.assertEqual(resp.submitted, 'pending')

class TestAutonomousSubmitGetAllSubmitted(unittest.TestCase):
    def test(self):
        resetDb()
        rest = ImagingInterface(isManual=False)

        # empty table
        self.assertIsNone(rest.getAllSubmittedTargets())

        # create one target
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId, 'standard', 'NE', 'circle', 'white', 'T', 'yellow')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = int(resp.headers['X-Class-Id'])
        modelResult = rest.getClassById(classId)
        self.assertIsNotNone(modelResult)
        targetId = modelResult.target

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
        modelResult = rest.getClassById(classId2)
        self.assertIsNotNone(modelResult)
        targetId2 = modelResult.target

        self.assertNotEqual(targetId, targetId2)

        self.assertIsNotNone(rest.postSubmitAllTargets())

        resp = rest.getAllSubmittedTargets()
        self.assertIsNotNone(resp)
        self.assertEqual(len(resp), 2)

        if resp[0].target == targetId:
            self.assertEqual(resp[0].crop_id, cropId)
            self.assertEqual(resp[1].crop_id, cropId2)
        elif resp[0].target == targetId2:
            self.assertEqual(resp[0].crop_id, cropId2)
        else:
            self.fail(msg="errrmmm. One of the returned target ids does not match what we submitted")