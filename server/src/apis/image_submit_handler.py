from flask import jsonify, request
from flask_restplus import Namespace, Resource, inputs
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous
from dao.manual_cropped_dao import ManualCroppedDAO
from dao.model.manual_cropped import manual_cropped
from dao.auvsi_odlc_file_dao import AuvsiOdlcDao
from dao.model.outgoing_target import outgoing_target
from config import defaultConfigPath
from apis.helper_methods import checkXManual, getClassificationDAO

api = Namespace('image/submit', description='Image submission helpers')

classificationParser = api.parser()
classificationParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True)

@api.route('/pend')
@api.expect(classificationParser)
class PendingSubmissionHandler(Resource):

    @api.doc(description='See all classifications pending submission. Grouped by targets')
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404:'Could not find any targets pending submission'})
    @api.expect(classificationParser)
    def get(self):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)

        results = dao.getPendingTargets()
        print(results)
        if results is None or not results or not results[0]:
            return {'message': 'Could not locate any distinct targets'}, 404

        jsonible = []
        for target in results:
            jsonible.append([ classification.toDict(exclude=('id',)) for classification in target ])

        return jsonify(jsonible)

@api.route('/<int:target_id>')
@api.doc(params={'target_id': 'Target id of the target group to get info on/submit'})
class SubmissionHandler(Resource):

    @api.doc(description='Get the submitted target for the given target id')
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404:'Could not find a SUBMITTED target with the given id (note targets will only be successfully retrieved if they have already been submitted by a POST)'})
    def get(self, target_id):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)

        resultTarget = dao.getSubmittedTarget(target_id)
        if resultTarget is None:
            return {'message': f'Failed to retrieve target {target_id}'}, 404
        
        return jsonify(resultTarget.toDict())

    @api.doc(description="""Submit the specified target. Returns that target information that was submitted 
        after averaging all the classification values for the target. The structure of the returned json depends
        on the type of target submitted""")
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404: 'Failed to find any targets with the given id waiting to be submitted'})
    def post(self, target_id):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)

        allTargetIds = dao.listTargetIds()

        if allTargetIds is None or not allTargetIds:
            return {'message': 'No classifications in outgoing (table empty)!'}, 404
        elif target_id not in allTargetIds:
            return {'message': 'Failed to find any targets with id {} to submit'.format(target_id)}, 404

        resultTarget = None
        try:
            resultTarget = dao.submitPendingTarget(target_id)
        except Exception as e:
            # something failed, make sure the target classifications are reset to 'unsubmitted'
            dao.resetTargetSubmissionStatus(target_id)
            raise # rethrow the same exception

        if resultTarget is None:
            return {'message': 'Something went wrong while trying to submit target {}'.format(target_id)}, 500

        prettyTargetOut = None
        try:
            prettyTargetOut = writeTargetToODLCFile(resultTarget, manual)
        except Exception as e:
            # something failed, make sure the target classifications are reset to 'unsubmitted'
            dao.resetTargetSubmissionStatus(target_id)
            raise # rethrow the same exception

        if prettyTargetOut is None and manual:
            dao.resetTargetSubmissionStatus(target_id)
            return {'message': 'Unable to find cropped_image entry with id: {}'.format(resultTarget.crop_id)}, 404

        # TODO: send to interop client?? 
        return jsonify(prettyTargetOut.toJson())
        

@api.route('/all')
class AllSubmissionHandler(Resource):

    @api.doc(description="""Submit all targets that do not yet have a 'submitted' status. Returns a list of targets 
        successfully submitted. As with single target submission, the structure of this json depends on the type of 
        target submitted""")
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404: 'Failed to find any targets waiting to be submitted'})
    def post(self):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)

        # cant really do any fancy checks like above, just go for submission:
        resultingTargets = dao.submitAllPendingTargets()

        if resultingTargets is None:
            return {'message': 'Either something went wrong, or there are not pending targets to submit'}, 404

        finalRet = []        
        for result in resultingTargets:
            try:
                prettyTargetOut = writeTargetToODLCFile(result, manual)
                if prettyTargetOut is None and manual:
                    dao.resetTargetSubmissionStatus(result.target)
                    print("WARN: Unable to find cropped_image entry with id {}".format(result.crop_id))
                if prettyTargetOut is not None:
                    finalRet.append(prettyTargetOut.toJson())
                    # TODO: send to interop client?? 
            except Exception as e:
                # something failed, make sure the target classifications are reset to 'unsubmitted'
                dao.resetTargetSubmissionStatus(result.target)
                raise # rethrow the same exception

        return jsonify(finalRet)


def writeTargetToODLCFile(target, manual):
    imagePath = None
    if manual:
        # then we need to get the cropped path
        croppedDao = ManualCroppedDAO(defaultConfigPath())
        croppedInfo = croppedDao.getImage(target.crop_id)

        if croppedInfo is None:
            return None
        imagePath = croppedInfo.cropped_path
    else:
        # autonomous we can just go
        imagePath = target.crop_path

    prettyTargetOut = outgoing_target(target, manual)
    auvsiDao = AuvsiOdlcDao()
    auvsiDao.addTarget(prettyTargetOut, imagePath)
    return prettyTargetOut