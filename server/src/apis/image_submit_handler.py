from flask import jsonify, request
from flask_restplus import Namespace, Resource, inputs, fields
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous

from dao.cropped_manual_dao import CroppedManualDAO
from dao.model.cropped_manual import cropped_manual
from dao.cropped_autonomous_dao import CroppedAutonomousDAO
from dao.model.cropped_autonomous import cropped_autonomous

from dao.auvsi_odlc_file_dao import AuvsiOdlcDao
from dao.submitted_target_dao import SubmittedTargetDAO
from dao.model.submitted_target import submitted_target
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

    @api.doc(description="""See all classifications pending submission. Grouped by targets. Unlike /all below which 
        returns a list of submitted targets. This returns a list of lists. The inner lists are classifications that are
        grouped into one target. Classifications for targets that have been submitted will not be shown here""")
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404:'Could not find any targets pending submission'})
    @api.expect(classificationParser)
    def get(self):
        manual = checkXManual(request)
        dao = getClassificationDAO(manual)

        results = dao.getPendingTargets()
        if results is None or not results or not results[0]:
            return {'message': 'Could not locate any targets pending submission'}, 404

        jsonible = []
        for target in results:
            jsonible.append([ classification.toDict() for classification in target ])

        return jsonify(jsonible)

@api.route('/<int:target_id>')
@api.doc(params={'target_id': 'Target id of the target group to get info on/submit'})
class SubmissionHandler(Resource):

    @api.doc(description="""Get the submitted target for the given target id. This will be an exact copy of 
        what was or is about to be submitted through interop to the judge server""")
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404:'Could not find a SUBMITTED target with the given id (note targets will only be successfully retrieved if they have already been submitted by a POST)'})
    def get(self, target_id):
        manual = checkXManual(request)
        dao = SubmittedTargetDAO(defaultConfigPath())

        resultTarget = dao.getTarget(target_id, (not manual))
        if resultTarget is None:
            return {'message': 'Failed to retrieve target {}'.format(target_id)}, 404

        # get the crop id from the path the target is using.
        # the crop_id is much more useful client-size than a crop_path
        outDict = resultTarget.toDict()
        croppedDao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        croppedImg = croppedDao.getImageWithCropPath(outDict['crop_path'])
        if croppedImg is not None:
            outDict['crop_id'] = croppedImg.crop_id
        
        return jsonify(outDict)

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
        content = request.get_json(silent=True) #silence error throwing if no json present

        # do a bunch of checks before we even think about submitting a target
        allTargetIds = dao.getAllTargetIDs()

        if allTargetIds is None or not allTargetIds:
            # is there even stuff in the table?
            return {'message': 'No classifications in outgoing (table empty)!'}, 404
        elif target_id not in allTargetIds:
            # is the target_id we are directed to submit in the table?
            return {'message': 'Failed to find any targets with id {} to submit'.format(target_id)}, 404

        if dao.isTargetSubmitted(target_id):
            # have we submitted said target already?
            return {'message': 'Target {} has already been submitted'.format(target_id)}, 400

        # if all the above checks pass, we know the specified target has yet to be 
        # submitted, and has data we can use for submission. Lets do it
        resultTarget = None
        try:
            resultTarget = dao.submitPendingTarget(target_id, content)
        except Exception as e:
            # something failed, make sure the target classifications are reset to 'unsubmitted'
            dao.resetTargetSubmissionStatus(target_id)
            raise # rethrow the same exception

        if resultTarget is None:
            dao.resetTargetSubmissionStatus(target_id) # something failed, reset to unsubmitted state
            return {'message': 'Something went wrong while trying to submit target {}'.format(target_id)}, 500

        submittedTarget = None
        targetSubmitDao = SubmittedTargetDAO(defaultConfigPath())
        try:
            submittedTarget = writeTargetToODLCFile(resultTarget, manual)
        except Exception as e:
            # something failed, make sure the target classifications are reset to 'unsubmitted'
            dao.resetTargetSubmissionStatus(target_id)
            raise # rethrow the same exception

        if submittedTarget is None:
            dao.resetTargetSubmissionStatus(target_id)
            print("ERR: Something went wrong trying to save the target to the ODLC format! Does the {} crop_id {} exist?".format('manual' if manual else 'autonomous', resultTarget.crop_id))
            return {'message': 'Something went wrong trying to save the target to the ODLC format! Does the {} crop_id {} exist?'.format('manual' if manual else 'autonomous', resultTarget.crop_id)}, 404
        else:
            # send to interop via the submitted_target table
            try:
                submittedTarget.submitted = None # get rid of classifications submission status
                targetSubmitDao.upsertTarget(submittedTarget)
            except Exception as e:
                dao.resetTargetSubmissionStatus(target_id)
                targetSubmitDao.removeTarget(target_id, (not manual))
                raise

        # we did it!
        return jsonify(submittedTarget.toAuvsiJson())

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

        targetSubmitDao = SubmittedTargetDAO(defaultConfigPath())
        finalRet = []
        for result in resultingTargets:
            try:
                submittedTarget = writeTargetToODLCFile(result, manual)
                if submittedTarget is None:
                    dao.resetTargetSubmissionStatus(result.target)
                    print("ERR: Something went wrong trying to save the target to the ODLC format! Does the {} crop_id {} exist?".format('manual' if manual else 'autonomous', result.crop_id))
                if submittedTarget is not None:
                    finalRet.append(submittedTarget.toAuvsiJson())
                    # add the target to our submitted_targets table where the ROS_handler 
                    # will grab it and ship it over to interop. Our work here is done :)
                    submittedTarget.submitted = None # get rid of classifications submission status
                    targetSubmitDao.upsertTarget(submittedTarget)
            except (Exception) as e:
                # something failed, make sure the target classifications are reset to 'unsubmitted'
                dao.resetTargetSubmissionStatus(result.target)
                targetSubmitDao.removeTarget(result.target, (not manual))
                raise # rethrow the same exception

        return jsonify(finalRet)

    @api.doc(description="""Get a list of all submitted targets. This list will be all targets that have been queued by 
        the client (autonomous or manual depending on the X-Manual header). The 'submitted' status for these targets will either be 
        'submitted' or 'pending' 'Pending' indicates that the client has posted the target for submission, and it is now waiting on 
        the interop handler to be submitted. Final submission usually only takes a second or two""")
    @api.expect(classificationParser)
    @api.doc(responses={200:'OK', 400:'X-Manual header not specified', 404: 'Failed to find any targets that have been submitted'})
    def get(self):
        manual = checkXManual(request)
        dao = SubmittedTargetDAO(defaultConfigPath())

        resultingClassifications = dao.getAllTargets(not manual)   
        if resultingClassifications is None or not resultingClassifications:
            return {'message': 'Failed to retrieve any submitted targets (Have any been submitted?)'}, 404

        resultingClassifications = [classification.toDict() for classification in resultingClassifications]

        croppedDao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())

        for target in resultingClassifications:
            # for each of these targets we need to try and get the crop_id for it from the crop_path
            # crop_id is much more useful client-side than crop_path
            croppedImg = croppedDao.getImageWithCropPath(target['crop_path'])
            if croppedImg is not None:
                target['crop_id'] = croppedImg.crop_id

        return jsonify(resultingClassifications)

def writeTargetToODLCFile(target, manual):
    imagePath = None

    # we have a crop_id from the classification generated for submission
    # now we need to take that and grab the cropped path
    croppedDao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
    croppedInfo = croppedDao.getImage(target.crop_id)

    if croppedInfo is None:
        return None
    imagePath = croppedInfo.cropped_path

    prettyTargetOut = submitted_target(outgoingManualOrAutonomous=target, autonomous_in=(not manual))
    prettyTargetOut.crop_path = imagePath

    auvsiDao = AuvsiOdlcDao()
    auvsiDao.addTarget(prettyTargetOut)
    return prettyTargetOut