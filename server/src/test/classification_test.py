from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.model.outgoing_manual import outgoing_manual
from config import defaultConfigPath
from test.test_helpers import truncateTable
import unittest

class TestAvgFunction(unittest.TestCase):
    def test(self):
        toAvg = [[0,1.0], [1,1.1], [2,1.4]]

        dao = OutgoingManualDAO(defaultConfigPath())
        self.assertAlmostEqual(dao.calcClmnAvg(toAvg, 1), 1.166666667)

class TestMostCommonFunction(unittest.TestCase):
    def test(self):
        toMostCommon = [[0, 'A'], [1, 'B'], [3, 'B'], [4, 'C']]
        dao = OutgoingManualDAO(defaultConfigPath())

        self.assertEqual(dao.findMostCommonValue(toMostCommon, 1), 'B')
        self.assertIsNone(dao.findMostCommonValue([[0, None], [1, None]], 1))

class TestManualClassificationInsert(unittest.TestCase):
    def test(self):
        # True for manual classification
        truncateTable('outgoing_manual')
        dao = OutgoingManualDAO(defaultConfigPath())

        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        # should now fail to insert a duplicate crop_id
        self.assertEqual(dao.addClassification(testIns), -1)
        self.assertIsNotNone(dao.getClassificationByUID(42))

class TestManualClassificationTargetBinning(unittest.TestCase):
    def test(self):
        truncateTable('outgoing_manual')

        dao = OutgoingManualDAO(defaultConfigPath())

        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'

        resultingId = dao.addClassification(testIns)
        self.assertNotEqual(resultingId, -1)
        insResult = dao.getClassification(resultingId)
        self.assertIsNotNone(insResult.target)
        self.assertNotEqual(insResult.target, -1)
        
        testIns.crop_id = 43
        resultingId = dao.addClassification(testIns)
        self.assertNotEqual(resultingId, -1)
        insResult2 = dao.getClassification(resultingId)
        self.assertEqual(insResult.target, insResult2.target)

        testIns.crop_id = 44
        testIns.alphanumeric = 'C'
        testIns.submitted = 'submitted' # this wont work -> it should still be unsubmitted
        resultingId = dao.addClassification(testIns)
        self.assertNotEqual(resultingId, -1)
        insResult3 = dao.getClassification(resultingId)
        self.assertNotEqual(insResult3.target, insResult2.target) 

        # the target id should change on this update
        testIns.crop_id = 42
        testIns.alphanumeric = 'C'
        testIns.submitted = 'unsubmitted'
        result = dao.updateClassificationByUID(testIns.crop_id, testIns.toDict())
        self.assertNotEqual(result.id, -1)
        self.assertEqual(result.target, insResult3.target)

class TestManualSubmitPendingTarget(unittest.TestCase):
    def test(self):
        # insert some targets
        dao = OutgoingManualDAO(defaultConfigPath())

        truncateTable('outgoing_manual')
        # see what it does with an empty table:
        self.assertIsNone(dao.submitPendingTarget(1))

        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.latitude = 40.111
        testIns.longitude = -111.111
        testIns.orientation = 'N'
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        self.assertIsNot(dao.addClassification(testIns), -1)

        testIns.crop_id = 43
        testIns.latitude = 40.222
        testIns.longitude = -111.222
        testIns.orientation = 'NE'
        testIns.background_color = 'orange'
        self.assertIsNot(dao.addClassification(testIns), -1)

        testIns.crop_id = 44
        testIns.latitude = 40.333
        testIns.longitude = -111.333
        testIns.alphanumeric_color = 'white'
        resultingId = dao.addClassification(testIns)
        self.assertIsNot(resultingId, -1)

        classResult = dao.getClassification(resultingId)
        submissionResult = dao.submitPendingTarget(classResult.target)

        self.assertAlmostEqual(submissionResult.latitude, 40.222)
        self.assertAlmostEqual(submissionResult.longitude, -111.222)
        self.assertEqual(submissionResult.orientation, 'NE')
        self.assertEqual(submissionResult.background_color, 'orange')
        self.assertEqual(submissionResult.alphanumeric_color, 'black')
        self.assertEqual(submissionResult.alphanumeric, 'A')
        self.assertEqual(submissionResult.shape, 'circle')

        # make sure that when we submit another classification that belongs
        # to the same target that its submitted state automatically goes to 
        # 'inherited_submission'
        testIns.crop_id = 45
        resultingId = dao.addClassification(testIns)
        self.assertNotEqual(resultingId, -1)
        classResult = dao.getClassification(resultingId)
        self.assertIsNotNone(classResult)
        self.assertEqual(classResult.submitted, 'inherited_submission')

class TestManualSubmitAllPendingTargets(unittest.TestCase):
    def test(self):
        truncateTable('outgoing_manual')
        dao = OutgoingManualDAO(defaultConfigPath())

        # see how it works on an empty table 
        self.assertIsNone(dao.submitAllPendingTargets())

        # this will insert records for 2 different targets from 3 classifications:
        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        dao.addClassification(testIns)
        
        testIns.crop_id = 43
        dao.addClassification(testIns)

        testIns.crop_id = 44
        testIns.alphanumeric = 'C'
        testIns.submitted = 'submitted' # this wont work -> it should still be unsubmitted
        dao.addClassification(testIns)
        
        result = dao.submitAllPendingTargets()
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 2) # should have 2 targets