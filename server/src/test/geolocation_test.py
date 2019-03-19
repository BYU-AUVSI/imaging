import unittest
from geolocation.geolocation import targetGeolocation

class TestClassConstructor(unittest.TestCase):
    def testConstructor(self):
        testLat = 1000
        testLon = 2000
        system = targetGeolocation(testLat, testLon)
        self.assertEqual(system.lat_gnd, testLat)
        self.assertEqual(system.lon_gnd, testLon)

class TestGeolocationAlgorithm(unittest.TestCase):
    def testArbitraryData(self):
        testLat = 1000
        testLon = 2000
        testMavLat = 250
        testMavLon = 120
        testHeight = 20
        testRoll = 20
        testPitch = 45
        testYaw = 95
        testTLCoorX = 200
        testTLCoorY = 200
        testBRCoorX = 400
        testBRCoorY = 400
        system = targetGeolocation(testLat, testLon)
        system.calculate_geolocation(testMavLat, testMavLon, testHeight, testRoll, testPitch, testYaw, testTLCoorX, testTLCoorY, testBRCoorX, testBRCoorY)
        self.assertEqual(sysytem.lat_mav, testMavLat)
        self.assertEqual(system.lon_mav, testMavLon)
        self.assertEqual(system.phi_deg, testRoll)
        self.assertEqual(system.theta_deg, testPitch)
        self.assertEqual(system.TopLeftX, testTLCoorX)
        self.assertEqual(system.TopLeftY, testTLCoorY)
        self.assertEqual(system.BottomRightX, testBRCoorX)
        self.assertEqual(system.BottomRightY, testBRCoorY)

    def testZeroStateData(self):
        testGndLat = 40.248471
        testGndLon = -111.645821
        testMavLat = 40.174375
        testMavLon = -111.655704
        testHeight = 16
        testRoll = 0
        testPitch = 0
        testYaw = 0
        testTLCoorX = 999
        testTLCoorY = 999
        testBRCoorX = 1001
        testBRCoorY = 1001
        system = targetGeolocation(testGndLat, testGndLon)
        targetLocation = system.calculate_geolocation(testMavLat, testMavLon, testHeight, testRoll, testPitch, testYaw, testTLCoorX, testTLCoorY, testBRCoorX, testBRCoorY)
        self.assertEqual(testMavLat, targetlocation[0])
        self.assertEqual(testMavLon, targetLocation[1])
        '''
        If the assertions above fail, it might just be a tolerance issue.
        '''

    def testRollvPitchYaw(self):
        testGndLat = 40.248471
        testGndLon = -111.645821
        testMavLat = 40.174375
        testMavLon = -111.655704
        testHeight = 16
        testRoll = 45
        testPitch = 0
        testYaw = 0
        testTLCoorX = 999
        testTLCoorY = 999
        testBRCoorX = 1001
        testBRCoorY = 1001
        system = targetGeolocation(testGndLat, testGndLon)
        targetLocationRoll = system.calculate_geolocation(testMavLat, testMavLon, testHeight, testRoll, testPitch, testYaw, testTLCoorX, testTLCoorY, testBRCoorX, testBRCoorY)
        testRoll = 0
        testPitch = 45
        testYaw = 90
        targetLocationPitchYaw = system.calculate_geolocation(testMavLat, testMavLon, testHeight, testRoll, testPitch, testYaw, testTLCoorX, testTLCoorY, testBRCoorX, testBRCoorY)
        self.assertEqual(targetLocationRoll[0], targetLocationPitchYaw[0])
        self.assertEqual(targetLocationRoll[1], targetLocationPitchYaw[1])
        '''
        If the assertions above fail, it might just be a tolerance issue.
        '''

    def generalTest(self):
        testGndLat = 40.248471
        testGndLon = -111.645821
        testMavLat = 40.246531
        testMavLon = -111.648298
        testHeight = 16
        testRoll = 0
        testPitch = 60
        testYaw = 90
        testTLCoorX = 1986
        testTLCoorY = 969
        testBRCoorX = 1988
        testBRCoorY = 971
        system = targetGeolocation(testGndLat, testGndLon)
        targetLocation = system.calculate_geolocation(testMavLat, testMavLon, testHeight, testRoll, testPitch, testYaw, testTLCoorX, testTLCoorY, testBRCoorX, testBRCoorY)
        self.assertTrue(targetLocation[0] - 40.2465 < .0005)
        self.assertTrue(targetLocation[1] - (-111.6483) < .0005)
