#! /usr/bin/env python

import os
from flask import jsonify
from dao.incoming_image_dao import IncomingImageDAO
from dao.model.incoming_image import incoming_image
from dao.model.outgoing_manual import outgoing_manual
from dao.outgoing_manual_dao import OutgoingManualDAO

"""
These tests are really hacky atm. Basically they just assume a whole bunch of crap
about how your current instance is setup (ie: what is and isn't in the database)

TOOD: Make them not crappy
"""

def getDefaultConfigFile():
    return os.path.dirname(os.path.realpath(__file__))  + '/../conf/config.ini'

def testIncomingImageGet():
    dao = IncomingImageDAO(getDefaultConfigFile())
    print("getting id 1...")
    resultingImg = dao.getImage(1)
    if resultingImg is None:
        print("id 1 doesn't exist!")
    else:
        print(resultingImg)

    print("getting id that prolly doesnt exist")
    resultingImg = dao.getImage(999999999)
    if resultingImg is None:
        print("id 999999999 doesn't exist!")
    else:
        print(resultingImg)

    dao.close()
    

def testClassificationInsert():
    # True for manual classification
    print('get classification dao')
    dao = OutgoingManualDAO(getDefaultConfigFile())

    testIns = outgoing_manual()
    testIns.crop_id = 42
    testIns.shape = 'circle'
    testIns.background_color = 'white'
    testIns.alphanumeric = 'A'
    testIns.alphanumeric_color = 'black'

    print('insert new')
    resultinID = dao.addClassification(testIns)
    print('id:: {}'.format(resultinID))
    assert resultinID != -1

    print("we should now fail to insert an identical image_id...")
    resultinID = dao.addClassification(testIns)
    assert resultinID == -1
    
    print('Get what we inserted by image_id:')
    result = dao.getClassificationByUID(42)
    assert result is not None
    print(result)
    
    print('done!')


def testClassificationTargetBinning():

    print('get manual classification dao')
    dao = OutgoingManualDAO(getDefaultConfigFile())

    testIns = outgoing_manual()
    testIns.crop_id = 42
    testIns.shape = 'circle'
    testIns.background_color = 'white'
    testIns.alphanumeric = 'A'
    testIns.alphanumeric_color = 'black'

    print('insert crop_id 42')
    resultingId = dao.addClassification(testIns)
    assert resultingId != -1
    insResult = dao.getClassification(resultingId)
    assert insResult.target != -1
    
    print('insert record that should be in same target bin as 42')
    testIns.crop_id = 43
    resultingId = dao.addClassification(testIns)
    assert resultingId != -1
    insResult2 = dao.getClassification(resultingId)
    assert insResult.target == insResult2.target

    print('insert record that belongs in a different target')
    testIns.crop_id = 44
    testIns.alphanumeric = 'C'
    resultingId = dao.addClassification(testIns)
    assert resultingId != -1
    insResult3 = dao.getClassification(resultingId)
    assert insResult3.target != insResult2.target

    print("updating uid 42")
    testIns.crop_id = 42
    testIns.alphanumeric = 'C'
    result = dao.updateClassificationByUID(testIns.crop_id, testIns.toDict())
    assert result.id != -1
    assert result.target == insResult3.target

def testDistinctManualClassification():
    print('get classification dao')
    dao = OutgoingManualDAO(getDefaultConfigFile())

    result = dao.getAllDistinct()

    printable = []
    for lst in result:
        printable.append([ classification.toDict(exclude=('id',)) for classification in lst ])

    print(jsonify(printable))

def main():
    # testIncomingImageGet()
    # testClassificationInsert()
    testClassificationTargetBinning()
    # testDistinctManualClassification()

if __name__ == '__main__':
    print("waddup")
    main()