from flask import request, abort
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from config import defaultConfigPath

def checkXManual(req):
    if 'X-Manual' not in req.headers:
        abort(400, 'Need to specify X-Manual header!')
    manual = req.headers.get('X-Manual')
    try:
        manual = manual.lower() == 'true'
    except:
        abort(400, 'Failed to interpret X-Manual header as boolean!')

    return manual


def getClassificationDAO(isManual):
    if isManual:
        return OutgoingManualDAO(defaultConfigPath())
    else:
        return OutgoingAutonomousDAO(defaultConfigPath())