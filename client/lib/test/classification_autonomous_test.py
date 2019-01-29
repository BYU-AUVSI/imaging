import unittest
from client_rest import ImagingInterface
from client_rest import Classification
from test.test_helpers import resetAutonomousDb as resetDb

class TestAutonomousCroppedPost(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface(isManual=False)
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
        
class TestAutonomousClassificationGetId(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface(isManual=False)
        ret = rest.getNextRawImage()
        self.assertIsNotNone(ret)

        # get stuff into the cropped and classification table
        resp = rest.postCroppedImage(ret[1], ret[0], [22, 22], [236, 236])
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # assert successful post to cropped
        cropId = int(resp.headers['X-Crop-Id'])
        classToPost = Classification(cropId, 'off_axis', 'NE', 'square', 'orange', 'T', 'black')
        resp = rest.postClass(classToPost)
        self.assertIsNotNone(resp)
        classId = int(resp.headers['X-Class-Id'])

        resp = rest.getClassById(classId)
        self.assertIsNotNone(resp)
        self.assertIsInstance(resp, Classification)
        # check data integrity from our previous post
        self.assertEqual(resp.crop_id, cropId)
        self.assertEqual(resp.id, classId)
        self.assertEqual(resp.type, 'off_axis')
        self.assertEqual(resp.orientation, 'NE')
        self.assertEqual(resp.shape, 'square')
        self.assertEqual(resp.background_color, 'orange')
        self.assertEqual(resp.alphanumeric, 'T')
        self.assertEqual(resp.alphanumeric_color, 'black')
        self.assertEqual(resp.submitted, 'unsubmitted')
        self.assertEqual(resp.description, '')
        self.assertIsNotNone(resp.target)


class TestAutonomousClassificationGetAll(unittest.TestCase):
    def test(self):
        resetDb()

        rest = ImagingInterface(isManual=False)
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

        if modelA.id == classId:
            #check model A
            self.assertEqual(modelA.crop_id, cropId)
            self.assertEqual(modelA.id, classId)
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
            self.assertEqual(modelB.id, classId2)
            self.assertEqual(modelB.type, 'off_axis')
            self.assertEqual(modelB.orientation, 'NE')
            self.assertEqual(modelB.shape, 'square')
            self.assertEqual(modelB.background_color, 'orange')
            self.assertEqual(modelB.alphanumeric, 'T')
            self.assertEqual(modelB.alphanumeric_color, 'black')
            self.assertEqual(modelB.submitted, 'unsubmitted')
            self.assertEqual(modelB.description, '')
            self.assertIsNotNone(modelB.target)
        elif modelA.id == classId2:
            #check model B
            self.assertEqual(modelB.crop_id, cropId)
            self.assertEqual(modelB.id, classId)
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
            self.assertEqual(modelA.id, classId2)
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