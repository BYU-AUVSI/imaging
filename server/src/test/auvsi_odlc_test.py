import unittest
import os
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.auvsi_odlc_file_dao import AuvsiOdlcDao
from dao.model.submitted_target import submitted_target


class TestManualAddClassification(unittest.TestCase):
    def test(self):
        testIns = outgoing_manual()
        testIns.crop_id = 42
        testIns.type = 'standard'
        testIns.orientation = 'NE'
        testIns.shape = 'star'
        testIns.background_color = 'blue'
        testIns.alphanumeric = 'A'
        testIns.alphanumeric_color = 'purple'
        testIns.latitude  = 40.11111
        testIns.longitude = -111.222222

        truncateTable('outgoing_manual')
        dao = OutgoingManualDAO(defaultConfigPath())

        id = dao.addClassification(testIns)
        self.assertNotEqual(id, -1)

        model = dao.getClassification(id)
        self.assertIsNotNone(model)

        classOut = dao.submitPendingTarget(model.target)
        self.assertIsNotNone(classOut)

        imgPath = os.path.dirname(os.path.realpath(__file__)) + '/assets/star.png'
        
        targetOut = submitted_target(outgoingManualOrAutonomous=classOut, autonomous=False)
        targetOut.crop_path = imgPath
        auvsiDao = AuvsiOdlcDao()
        auvsiDao.addTarget(targetOut)


