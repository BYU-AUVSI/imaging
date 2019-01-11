from flask import jsonify, request
from flask_restplus import Namespace, Resource, inputs, fields
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

# for documentation purposes. Defines potential input for target submit POST
classificationModel = api.model('Target Classification Choices', {
    'crop_id': fields.Integer(reqired=False, description='[optional] CLASSIFICATION_ID, of the classification that has the crop_id to use for target submission', example=12),
    'orientation': fields.Integer(required=False, description='[optional] Classification id of the classification that has the orientation value we want to use for target submission', example=14),
    'background_color': fields.Integer(required=False, description='[optional] Classification id of the classification that has the background_color value we want to use for target submission', example=15),
    'alphanumeric_color': fields.Integer(required=False, description='[optional] Classification id of the classification that has the alphanumeric_color value we want to use for target submission', example=13),
    'description': fields.Integer(required=False, description='[optional][emergent-only] Classification id of the classification with the description we want to use for target submission', example=15)
})

classificationParser = api.parser()
classificationParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help="Specify whether this request is for manual or autonomous submissions")


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
            return {'message': 'Failed to retrieve target {}'.format(target_id)}, 404
        
        return jsonify(resultTarget.toDict(exclude=("id",)))

    @api.doc(description="""Submit the specified target. Returns that target information that was submitted 
        after averaging all the classification values for the target. The structure of the returned json depends
        on the type of target submitted.\n
        NOTE: Optionally, this request also accepts a json payload, where you can specify the classification_ids of target features you want to use.
        ie: if the orientation held by classification 12 is correct for this target, you can tell the server to not average values,
        but to instead just use classification 12's orientation value for the final target submission. The payload would look something like this:\n
        {
            "crop_id": 13,
            "orientation": 12,
            "background_color": 13,
            "alphanumeric_color": 14,
            "description": 15
        }
        Note all these values are optional. and 'crop_id' is in fact the CLASSIFICATION_ID that has the crop_id you want to use (confusing, but consistent). 
        Description will only be used for the emergent object.""")
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404: 'Failed to find any targets with the given id waiting to be submitted'})
    def post(self, target_id):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)
        content = request.get_json()

        allTargetIds = dao.listTargetIds()

        if allTargetIds is None or not allTargetIds:
            return {'message': 'No classifications in outgoing (table empty)!'}, 404
        elif target_id not in allTargetIds:
            return {'message': 'Failed to find any targets with id {} to submit'.format(target_id)}, 404

        resultTarget = None
        try:
            resultTarget = dao.submitPendingTarget(target_id, content)
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

    @api.doc(description="""Get a list of all submitted targets. This information will be slightly 
        different from a classification. Submitted targets take averages of all the classifications 
        for that target into account for final submission""")
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404: 'Failed to find any targets that have been submitted'})
    def get(self):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)

        resultingClassifications = dao.getSubmittedTargets()   
        if resultingClassifications is None or not resultingClassifications:
            return {'message': 'Failed to retrieve any submitted targets (Have any been submitted?)'}, 404

        resultingClassifications = [classification.toDict(exclude=('id',)) for classification in resultingClassifications]

        return jsonify(resultingClassifications)

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