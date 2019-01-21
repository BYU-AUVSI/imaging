import unittest
from client_rest import ImagingInterface
from client_rest import ImageInfo
from PIL import Image
from test.test_helpers import resetManualDb

class TestGetNextRawImageManual(unittest.TestCase):
    def test(self):
        resetManualDb()

        rest = ImagingInterface()

        ret = rest.getNextRawImage()

        self.assertIsNotNone(ret)
        self.assertEqual(len(ret), 2)
        self.assertIsNotNone(ret[0])
        self.assertIsNotNone(ret[1])
        self.assertIsNotNone(ret[0].size) # (width, height) of image

        self.assertNotEqual(id, -1)
