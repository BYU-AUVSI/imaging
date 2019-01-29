from flask import request, jsonify, abort, make_response
from flask_restplus import Namespace, Resource, inputs, fields
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_manual import outgoing_manual
from config import defaultConfigPath
from apis.helper_methods import checkXManual

api = Namespace('image/class/manual', description='Image classification calls for manual clients route through here')

cropIDParser = api.parser()
cropIDParser.add_argument('X-Crop-Id', location='headers', type=int, required=True, help='The cropped_id this classification is associated with')
cropIDParser.add_argument('X-Manual', location='headers', type=int, required=True, help='Specify whether this is a manual request (True) or autonomous (False)')

xManParser = api.parser()
xManParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)') 

# for documentation purposes. Defines the response for some of the methods below
classificationModel = api.model('Manual Classification', {
    'target': fields.Integer(required=False, description='which target number this is. This field is automatically managed by the server. It groups classifications together based on alphanumeric, shape and type', example=1),
    'crop_id': fields.Integer(reqired=True, description='Id of the cropped image this classification originally comes from', example=123),
    'type': fields.String(required=False, description='Classification type(standard, off_axis, or emergent)', example="standard"),
    'latitude': fields.Float(required=False, description='Latitude coordinate of object', example=40.246354),
    'longitude': fields.Float(required=False, description='longitude coordinate of object', example=-111.647553),
    'orientation': fields.String(required=False, description='Describes the heading/orientation of the letter(N, NE, E, etc...)', example='N'),
    'shape': fields.String(required=False, description='The shape of the object for standard/off_axis types(circle, triangle, square, etc...)', example='circle'),
    'background_color': fields.String(required=False, description='Color of the background the letter is on for standard/off_axis types(white, black, orange, etc...)', example='orange'),
    'alphanumeric': fields.String(required=False, description='The letter within the object for standard/off_axis types(A, B, C, etc...)', example='A'),
    'alphanumeric_color': fields.String(required=False, description='Color of the letter for standard/off_axis types(white, black, orange, etc...)', example='black'),
    'description': fields.String(required=False, description='For the emergent type, description of what it is'),
    'submitted': fields.String(required=False, description='Whether or not this particular classification has been submitted to the judges. Defaults to "unsubmitted". "submitted" indicates that this classification has been submitted to the judges. "inherited_submission" indicates that another classification for the same target (ie: another image of the same target) has already been submitted, and its therefore unnecessary to submit this one.', example='unsubmitted')
})

classificationSubmission = api.model('Submit Manual Classification', {
    'type': fields.String(required=False, description='Classification type(standard, off_axis, or emergent)', example="standard"),
    'crop_id': fields.Integer(reqired=True, description='Id of the cropped image this classification originally comes from', example=123),
    'latitude': fields.Float(required=False, description='Latitude coordinate of object', example=40.246354),
    'longitude': fields.Float(required=False, description='longitude coordinate of object', example=-111.647553),
    'orientation': fields.String(required=False, description='Describes the heading/orientation of the letter(N, NE, E, etc...)', example='N'),
    'shape': fields.String(required=False, description='The shape of the object for standard/off_axis types(circle, triangle, square, etc...)', example='circle'),
    'background_color': fields.String(required=False, description='Color of the background the letter is on for standard/off_axis types(white, black, orange, etc...)', example='orange'),
    'alphanumeric': fields.String(required=False, description='The letter within the object for standard/off_axis types(A, B, C, etc...)', example='A'),
    'alphanumeric_color': fields.String(required=False, description='Color of the letter for standard/off_axis types(white, black, orange, etc...)', example='black'),
    'description': fields.String(required=False, description='For the emergent type, description of what it is')
})

@api.route('/all')
class AllClassificationsHandler(Resource):

    @api.doc(description='Get all the manual classifications currently on the server')
    @api.response(200, 'OK', [classificationModel])
    @api.doc(responses={404:'No classifications (table empty)'})
    @api.expect(xManParser)
    def get(self):
        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        outgoingList = []
        dao = OutgoingManualDAO(defaultConfigPath()) if manual else OutgoingAutonomousDAO(defaultConfigPath())
        outgoingList = dao.getAll()

        if not outgoingList:
            return {'message': 'Outgoing table is empty!'}, 404

        exportable = [ classification.toDict() for classification in outgoingList ]
        return jsonify(exportable)


@api.route("/")
class ClassifiedImageHandler(Resource):
    @api.doc(description='Automatically add a new classifcation to the server')
    @api.doc(responses={200:'OK', 400:'Improper image post', 500: 'Something failed server-side'})
    @api.expect(classificationSubmission)
    @api.header('X-Class-Id', 'Classification ID of the image if successfully inserted. This WILL be different from the Crop-ID provided in the request')
    def post(self):
        prevId = -1
        if 'X-Crop-Id' in request.headers:
            prevId = request.headers.get('X-Crop-Id')
        else:
            abort(400, "Need to specify header 'X-Crop-Id'!")

        dao = OutgoingManualDAO(defaultConfigPath())

        outgoingIn = outgoing_manual(json=request.get_json())
        outgoingIn.crop_id = prevId
        resultingId = dao.upsertClassification(outgoingIn)
        
        if resultingId == -1:
            return {'message': 'Failed to insert classification into outgoing table'}, 500

        response = make_response(jsonify({'message': 'success!', 'id': resultingId}))
        response.headers['X-Class-Id'] = resultingId
        return response


@api.route('/<int:class_id>')
@api.doc(params={'class_id': 'Classification ID of the classification entry to update or get info on'}, required=True)
class SpecificClassificationHandler(Resource):

    @api.doc(description='Get the classification for the given id')
    @api.response(200, 'OK', classificationModel)
    @api.doc(responses={404:'Could not find classification with given ID'})
    def get(self, class_id):

        dao = OutgoingManualDAO(defaultConfigPath())
        result = dao.getClassification(class_id)
        if result is None:
            return {'message': 'Failed to locate classification with id {}'.format(class_id)}, 404
        return jsonify(result.toDict())


    @api.doc(description='Update information for the specified classification entry')
    @api.response(200, 'OK', classificationModel)
    @api.doc(body=classificationSubmission)
    @api.doc(responses={404:'Could not find classification with given ID'})
    def put(self, class_id):

        dao = OutgoingManualDAO(defaultConfigPath())
        result = dao.updateClassification(class_id, request.get_json())
        if result is None:
            return {'message': 'No image with id {} found with a classification to update or your input was invalid (or was there a server error?)'.format(class_id)}, 404
        else:
            return jsonify(result.toDict())

    @api.doc(description='Delete the given classification from the ougoing table')
    @api.doc(responses={404:'Could not find classification with given ID', 500: 'The delete didnt work for some reason'})
    def delete(self, class_id):
        dao = OutgoingManualDAO(defaultConfigPath())

        if dao.getClassification(class_id) is None:
            return {'message': 'Couldnt find classification with id {}'.format(class_id)}, 404

        result = dao.removeClassification(class_id)
        if not result:
            return {'message': 'Something went wrong while trying to delete id {} (was it delete by someone else first??)'.format(class_id)}, 500
        
        return {'message': 'success!', 'id': class_id}