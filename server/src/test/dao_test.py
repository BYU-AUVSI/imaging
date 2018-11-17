#! /usr/bin/env python

import os
from dao.incoming_image_dao import IncomingImageDAO
from dao.model.incoming_image import incoming_image

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
    

def main():
    testIncomingImageGet()

if __name__ == '__main__':
    print("waddup")
    main()