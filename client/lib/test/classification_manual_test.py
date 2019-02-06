import unittest
from client_rest import ImagingInterface
from client_rest import Classification
from test.test_helpers import resetManualDb as resetDb

class TestManualClassificationPost(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface()
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
        self.assertIsNotNone(classId)
        self.assertNotEqual(int(classId), -1)
        
class TestManualClassificationGetId(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface()
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)

        # get stuff into the cropped and classification table
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId, 'standard', 'NE', 'circle', 'white', 'T', 'yellow')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = int(resp.headers['X-Class-Id'])

        resp = rest.getClassById(classId)
        self.assertIsNotNone(resp)
        self.assertIsInstance(resp, Classification)
        # check data integrity from our previous post
        self.assertEqual(resp.crop_id, cropId)
        self.assertEqual(resp.class_id, classId)
        self.assertEqual(resp.type, 'standard')
        self.assertEqual(resp.orientation, 'NE')
        self.assertEqual(resp.shape, 'circle')
        self.assertEqual(resp.background_color, 'white')
        self.assertEqual(resp.alphanumeric, 'T')
        self.assertEqual(resp.alphanumeric_color, 'yellow')
        self.assertEqual(resp.submitted, 'unsubmitted')
        self.assertEqual(resp.description, '')
        self.assertIsNotNone(resp.target)

class TestManualClassificationGetAll(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface()
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)

        # get stuff into the cropped and classification table
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId, 'standard', 'NE', 'circle', 'white', 'T', 'yellow')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = int(resp.headers['X-Class-Id'])

        # post a second crop and classification:
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId2 = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId2, 'off_axis', 'NE', 'square', 'orange', 'T', 'black')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId2 = int(resp.headers['X-Class-Id'])


        # get all:
        resp = rest.getAllClass()
        self.assertIsNotNone(resp)
        self.assertEqual(len(resp), 2)


        modelA = resp[0]
        modelB = resp[1]

        if modelA.class_id == classId:
            #check model A
            self.assertEqual(modelA.crop_id, cropId)
            self.assertEqual(modelA.class_id, classId)
            self.assertEqual(modelA.type, 'standard')
            self.assertEqual(modelA.orientation, 'NE')
            self.assertEqual(modelA.shape, 'circle')
            self.assertEqual(modelA.background_color, 'white')
            self.assertEqual(modelA.alphanumeric, 'T')
            self.assertEqual(modelA.alphanumeric_color, 'yellow')
            self.assertEqual(modelA.submitted, 'unsubmitted')
            self.assertEqual(modelA.description, '')
            self.assertIsNotNone(modelA.target)

            # check model B
            self.assertEqual(modelB.crop_id, cropId2)
            self.assertEqual(modelB.class_id, classId2)
            self.assertEqual(modelB.type, 'off_axis')
            self.assertEqual(modelB.orientation, 'NE')
            self.assertEqual(modelB.shape, 'square')
            self.assertEqual(modelB.background_color, 'orange')
            self.assertEqual(modelB.alphanumeric, 'T')
            self.assertEqual(modelB.alphanumeric_color, 'black')
            self.assertEqual(modelB.submitted, 'unsubmitted')
            self.assertEqual(modelB.description, '')
            self.assertIsNotNone(modelB.target)
        elif modelA.class_id == classId2:
            #check model B
            self.assertEqual(modelB.crop_id, cropId)
            self.assertEqual(modelB.class_id, classId)
            self.assertEqual(modelB.type, 'standard')
            self.assertEqual(modelB.orientation, 'NE')
            self.assertEqual(modelB.shape, 'circle')
            self.assertEqual(modelB.background_color, 'white')
            self.assertEqual(modelB.alphanumeric, 'T')
            self.assertEqual(modelB.alphanumeric_color, 'yellow')
            self.assertEqual(modelB.submitted, 'unsubmitted')
            self.assertEqual(modelB.description, '')
            self.assertIsNotNone(modelB.target)

            # check model A
            self.assertEqual(modelA.crop_id, cropId2)
            self.assertEqual(modelA.class_id, classId2)
            self.assertEqual(modelA.type, 'off_axis')
            self.assertEqual(modelA.orientation, 'NE')
            self.assertEqual(modelA.shape, 'square')
            self.assertEqual(modelA.background_color, 'orange')
            self.assertEqual(modelA.alphanumeric, 'T')
            self.assertEqual(modelA.alphanumeric_color, 'black')
            self.assertEqual(modelA.submitted, 'unsubmitted')
            self.assertEqual(modelA.description, '')
            self.assertIsNotNone(modelA.target)
        else:
            self.fail()
        # check data integrity from our previous post
        
class TestManualClassificationDelete(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface()
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
        self.assertIsNotNone(classId)
        self.assertNotEqual(int(classId), -1)

        resp = rest.deleteClass(classId)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200)

        # we should fail to get the classification
        self.assertIsNone(rest.getClassById(classId))

class TestManualClassificationUpdate(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface()
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
        self.assertIsNotNone(classId)
        self.assertNotEqual(int(classId), -1)

        # should fail to update stuff that doesn't exist
        stuffToUpdate = Classification(None, None, "SE", "square", None, "Y", "purple")
        self.assertIsNone(rest.updateClass(int(classId) + 20, stuffToUpdate))

        resp = rest.updateClass(classId, stuffToUpdate)
        
        # confirm update claims success
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200)
        
        # confirm update was acutlaly successful
        updated = rest.getClassById(classId)
        self.assertIsNotNone(updated)
        self.assertEqual(updated.crop_id, cropId)
        self.assertEqual(updated.type, 'standard')
        self.assertEqual(updated.orientation, 'SE')
        self.assertEqual(updated.shape, 'square')
        self.assertEqual(updated.background_color, 'white')
        self.assertEqual(updated.alphanumeric, "Y")
        self.assertEqual(updated.alphanumeric_color, "purple")
