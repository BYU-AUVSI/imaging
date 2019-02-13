import unittest
from client_rest import ImagingInterface
from client_rest import StateMeasurement

# two timestamps inserted for testing before these were run
lowerTs = 1547453775.2
upperTs = lowerTs + 10000

class TestGetStateByTS(unittest.TestCase):
    def test(self):
        rest = ImagingInterface(host='127.0.0.1', port='5000', isDebug=False)

        # feed a timestamp that has no way of existing in the table
        result = rest.getStateByTs(12345.6)
        self.assertIsNone(result)
        
        # test a timestamp that should be exactly equal to
        # the first record inserted by setup_db_for_client_tests.py
        result = rest.getStateByTs(lowerTs)
        self.assertIsNotNone(result)
        lowerId = result.id
        self.assertNotEqual(lowerId, -1)
        self.assertEqual(result.roll, 40.111)
        self.assertEqual(result.pitch, 111.222)
        self.assertEqual(result.yaw, 12.3)
        self.assertEqual(result.time_stamp, lowerTs)

        # something right in between the two inserted timestamps
        # will still return the lower
        result = rest.getStateByTs((lowerTs + upperTs) / 2)
        self.assertIsNotNone(result)
        self.assertEqual(lowerId, result.id)
        
        result = rest.getStateByTs(upperTs + 1.0)
        self.assertIsNotNone(result)
        self.assertNotEqual(lowerId, result.id)
        self.assertEqual(result.time_stamp, upperTs)
        self.assertEqual(result.roll, 40.222)
        self.assertEqual(result.pitch, 111.333)
        self.assertEqual(result.yaw, 34.5)

class TestGetStateById(unittest.TestCase):
    def test(self):
        rest = ImagingInterface(host='127.0.0.1', port='5000', isDebug=False)
        
        self.assertIsNone(rest.getStateById(-1))

        # get id for the lower TS value that we know
        result = rest.getStateByTs(lowerTs)
        self.assertIsNotNone(result)
        lowerId = result.id

        result = rest.getStateByTs(upperTs)
        self.assertIsNotNone(result)
        upperId = result.id

        result = None
        result = rest.getStateById(lowerId)
        self.assertIsNotNone(result)
        self.assertNotEqual(lowerId, -1)
        self.assertEqual(result.roll, 40.111)
        self.assertEqual(result.pitch, 111.222)
        self.assertEqual(result.yaw, 12.3)
        self.assertEqual(result.time_stamp, lowerTs)

        result = rest.getStateById(upperId)
        self.assertIsNotNone(result)
        self.assertNotEqual(lowerId, result.id)
        self.assertEqual(result.time_stamp, upperTs)
        self.assertEqual(result.roll, 40.222)
        self.assertEqual(result.pitch, 111.333)
        self.assertEqual(result.yaw, 34.5)