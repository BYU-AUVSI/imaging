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
    testIns.image_id = 42
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
    testDistinctManualClassification()

if __name__ == '__main__':
    print("waddup")
    main()