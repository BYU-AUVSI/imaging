import os, unittest
from config import defaultConfigPath
from test.test_helpers import truncateTable
from dao.incoming_image_dao import IncomingImageDAO
from dao.manual_cropped_dao import ManualCroppedDAO
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.model.incoming_image import incoming_image
from dao.model.manual_cropped import manual_cropped
from dao.model.outgoing_manual import outgoing_manual


def setupClientRestTestDb():
    # truncate all the tables:
    truncateTable('incoming_image')
    truncateTable('incoming_gps')
    truncateTable('incoming_state')
    truncateTable('manual_cropped')
    truncateTable('outgoing_manual')
    truncateTable('outgoing_autonomous')

    # figure out the absolute path to our raw test image:
    rawImg = os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + '/assets/rawFrame.jpg')
    # figure out the path for our cropped test img
    cropImg = os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + '/assets/star.png')

    # setup incoming image table first
    imageIds= setupIncomingImageTable(rawImg)
    # use ids from incoming_image to setup manual_cropped table
    cropIds = setupManualCroppedTable(cropImg, imageIds)
    setupManualOutgoingTable(cropIds)


def setupIncomingImageTable(rawImgPath):
    dao = IncomingImageDAO(defaultConfigPath())
    incModel = incoming_image()
    incModel.focal_length = 16.0
    incModel.time_stamp = 1547453775.2
    incModel.image_path = rawImgPath
    incModel.manual_tap = False
    incModel.autonomous_tap = False
    incImageId1 = dao.addImage(incModel)
    assert incImageId1 != -1

    # just gonna use a slightly different number so we can tell the difference between the two inserts
    incModel.focal_length = 17.0 
    incImageId2 = dao.addImage(incModel)
    assert incImageId2 != -1
    return (incImageId1, incImageId2)

def setupManualCroppedTable(cropImgPath, imageIds):
    dao = ManualCroppedDAO(defaultConfigPath())
    incModel = manual_cropped()
    incModel.image_id   = imageIds[0]
    incModel.time_stamp = 1547453775.2
    incModel.cropped_path = cropImgPath
    incModel.crop_coordinate_tl = "(12, 34)"
    incModel.crop_coordinate_br = "(56, 78)"
    cropId1 = dao.upsertCropped(incModel)
    assert cropId1 != -1

    incModel.crop_coordinate_tl = "(90, 110)"
    incModel.crop_coordinate_br = "(120, 130)"
    cropId2 = dao.upsertCropped(incModel)
    assert cropId2 != -1

    incModel.image_id = imageIds[1]
    incModel.crop_coordinate_tl = "(140, 150)"
    incModel.crop_coordinate_br = "(160, 170)"
    cropId3 = dao.upsertCropped(incModel)
    assert cropId3 != -1
    return (cropId1, cropId2, cropId3)

def setupManualOutgoingTable(cropImgIds):
    dao = OutgoingManualDAO(defaultConfigPath())
    testIns = outgoing_manual()
    testIns.crop_id = cropImgIds[0]
    testIns.type = 'standard'
    testIns.shape = 'circle'
    testIns.background_color = 'white'
    testIns.alphanumeric = 'A'
    testIns.alphanumeric_color = 'black'
    assert dao.addClassification(testIns) != -1

    testIns.crop_id = cropImgIds[1]
    testIns.alphanumeric_color = 'orange'
    testIns.background_color = 'gray'
    assert dao.addClassification(testIns) != -1

    testIns.crop_id = cropImgIds[2]
    testIns.shape = 'triangle'
    testIns.alphanumeric = 'B'
    testIns.background_color = 'blue'
    assert dao.addClassification(testIns) != -1

