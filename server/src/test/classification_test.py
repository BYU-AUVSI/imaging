import unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous

class TestOutgoingManualConnection(unittest.TestCase):
    def test(self):
        dao = OutgoingManualDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        self.assertIsNotNone(dao.conn)

class TestOutgoingAutonomousConnection(unittest.TestCase):
    def test(self):
        dao = OutgoingAutonomousDAO(defaultConfigPath())
        self.assertIsNotNone(dao)
        self.assertIsNotNone(dao.conn)

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

class TestManualClassificationAdd(unittest.TestCase):
    def test(self):
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

class TestAutonomousClassificationAdd(unittest.TestCase):
    def test(self):
        truncateTable('outgoing_autonomous')
        dao = OutgoingAutonomousDAO(defaultConfigPath())

        testIns = outgoing_autonomous()
        testIns.image_id = 123
        testIns.crop_path = '/am/i/even/a/real/path/idk/anymore.jpg'
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        resultingId = dao.addClassification(testIns)
        self.assertNotEqual(resultingId, -1)

        # should be able to insert a duplicate record
        self.assertNotEqual(dao.addClassification(testIns), -1)
        self.assertIsNotNone(dao.getClassificationByUID(resultingId))

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

class TestManualClassificationRemoval(unittest.TestCase):
    def test(self):
        # insert some targets
        dao = OutgoingManualDAO(defaultConfigPath())

        truncateTable('outgoing_manual')

        # make sure if fails when we try and remove on empty table:
        deleteResult = dao.removeClassification(100)
        self.assertFalse(deleteResult)
        
        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.latitude = 40.111
        testIns.longitude = -111.111
        testIns.orientation = 'N'
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        resultingId = dao.addClassification(testIns)
        self.assertIsNot(resultingId, -1)

        testIns.crop_id = 44
        otherId = dao.addClassification(testIns)
        self.assertNotEqual(otherId, -1)

        # make sure it doesn't delete everything:
        deleteResult = dao.removeClassification(otherId + 5) # give bogus id that should fail
        self.assertFalse(deleteResult)
        self.assertEqual(len(dao.getAll()), 2) # make sure there's still 2 records

        deleteResult = dao.removeClassification(resultingId)
        self.assertTrue(deleteResult)
        self.assertEqual(len(dao.getAll()), 1) # should still be 1 record left
        self.assertIsNotNone(dao.getClassification(otherId)) # make sure the otherId wasn't deleted on accident

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

class TestManualSubmitPendingTargetWithClassSpecs(unittest.TestCase):
    def test(self):
        truncateTable('outgoing_manual')
        dao = OutgoingManualDAO(defaultConfigPath())

        # at this point we should've already passed a bunch of tests
        # relating to target submission above, so we can just test the
        # classification specification feature here

        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.latitude = 40.111
        testIns.longitude = -111.111
        testIns.orientation = 'N'
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'black'
        firstClass = dao.addClassification(testIns)
        self.assertNotEqual(firstClass, -1)

        testIns.crop_id = 43
        testIns.latitude = 40.222
        testIns.longitude = -111.222
        testIns.orientation = 'NE'
        testIns.background_color = 'orange'
        secondClass = dao.addClassification(testIns)
        self.assertNotEqual(secondClass, -1)

        testIns.crop_id = 44
        testIns.latitude = 40.333
        testIns.longitude = -111.333
        testIns.alphanumeric_color = 'white'
        thirdClass = dao.addClassification(testIns)
        self.assertNotEqual(thirdClass, -1)

        specs = {'orientation': secondClass,
                'crop_id': firstClass,
                'alphanumeric_color':thirdClass}

        classResult = dao.getClassification(secondClass)
        submissionResult = dao.submitPendingTarget(classResult.target, specs)

        self.assertIsNotNone(submissionResult)

        self.assertEqual(submissionResult.orientation, 'NE')
        self.assertEqual(submissionResult.crop_id, 42)
        self.assertEqual(submissionResult.alphanumeric_color, 'white')
        self.assertEqual(submissionResult.background_color, 'orange')
        self.assertEqual(submissionResult.alphanumeric, 'A')

        truncateTable('outgoing_manual')
        ############################################
        # test what happens when we put garbage in specs:
        ############################################
        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.latitude = 40.111
        testIns.longitude = -111.111
        testIns.orientation = 'S'
        testIns.shape = 'circle'
        testIns.background_color = 'white'
        testIns.alphanumeric = 'Q'
        testIns.alphanumeric_color = 'black'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        testIns.crop_id = 43
        testIns.latitude = 40.222
        testIns.longitude = -111.222
        testIns.orientation = 'W'
        testIns.background_color = 'orange'
        secondClass = dao.addClassification(testIns)
        self.assertNotEqual(secondClass, -1)

        testIns.crop_id = 44
        testIns.latitude = 40.333
        testIns.longitude = -111.333
        testIns.alphanumeric_color = 'white'
        self.assertNotEqual(dao.addClassification(testIns), -1)

        specs = {'orientation': secondClass,
                'crop_id': None,
                'alphanumeric_color':'wasdf'}

        classResult = dao.getClassification(secondClass)
        submissionResult = dao.submitPendingTarget(classResult.target, specs)

        # Even though we fed a bunch of garbage in specs, submission should
        # still succeed, defaulting to most common value, for the garbage stuff
        self.assertIsNotNone(submissionResult)

        self.assertEqual(submissionResult.orientation, 'W')
        self.assertEqual(submissionResult.alphanumeric_color, 'black')
        self.assertEqual(submissionResult.background_color, 'orange')
        self.assertEqual(submissionResult.alphanumeric, 'Q')

