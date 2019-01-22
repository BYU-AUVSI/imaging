import unittest
from client_rest import ImagingInterface

class TestServerPing(unittest.TestCase):
    def test(self):
        rest = ImagingInterface()
        self.assertIsNotNone(rest)
        self.assertTrue(rest.ping())

        # go to the wrong port number to verify ping fails
        rest = ImagingInterface(port="6000")
        self.assertIsNotNone(rest)
        self.assertFalse(rest.ping())