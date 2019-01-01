from flask import request, jsonify, abort, make_response
from flask_restplus import Namespace, Resource, inputs, fields
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous
from apis.helper_methods import checkXManual, getClassificationDAO

api = Namespace('image/class', description='Imaging classification calls route through here')

classificationParser = api.parser()
classificationParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True)

# for documentation purposes. Defines the response for some of the methods below
classificationModel = api.model('Classification', {
    'target': fields.Integer(required=False, description='which target number this is. This field is automatically managed by the server. It groups classifications together based on alphanumeric, shape and type', example=1),
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

@api.route('/all')
class AllClassificationsHandler(Resource):

    @api.doc(description='Get all the completed classifications for the corresponding process type (Manual or Autonomous)')
    @api.response(200, 'OK', [classificationModel])
    @api.doc(responses={400:'X-Manual header not specified', 404:'Outgoing classification table is empty'})
    def get(self):
        # Input Validation::
        manual = checkXManual(request)

        outgoingList = []
        dao = getClassificationDAO(manual)
        outgoingList = dao.getAll()

        if not outgoingList:
            return {'message': 'Outgoing table is empty!'}, 404

        exportable = [ classification.toDict() for classification in outgoingList ]
        return jsonify(exportable)


@api.route("/")
class ClassifiedImageHandler(Resource):
    @api.doc(description='Automatically add a new classifcation to the server')
    @api.doc(responses={200:'OK', 400:'Improper image post', 500: 'Something failed server-side'})
    @api.header('X-Class-Id', 'Crop ID of the image if successfully inserted. This WILL be different from the Image-ID provided in the request')
    def post(self):
        manual = checkXManual(request)

        prevId = -1
        if 'X-Prev-Id' in request.headers:
            prevId = request.headers.get('X-Prev-Id')
        else:
            abort(400, "Need to specify header 'X-Prev-Id'!")

        dao = getClassificationDAO(manual)
        if manual:
            outgoingIn = outgoing_manual(json=api.payload)
            outgoingIn.crop_id = prevId
            resultingId = dao.upsertClassification(outgoingIn)
        else:
            # since image_id in outgoing_autonomous is not unique like 
            # crop_id is for outgoing_manual, we just do a straight insert here
            outgoingIn = outgoing_autonomous(json=api.payload)
            outgoingIn.image_id = prevId
            resultingId = dao.addClassification(outgoingIn)
        
        if resultingId == -1:
            return {'message': 'Failed to insert classification into outgoing table'}, 500

        response = make_response(jsonify({'message': 'success!', 'id': resultingId}))
        response.headers['X-Class-Id'] = resultingId
        return response


@api.route('/<int:class_id>')
@api.doc(params={'class_id': 'Classification ID of the classification entry to update or get info on'}, required=True)
class SpecificClassificationHandler(Resource):

    @api.doc(description='Get the classification for the given id')
    @api.expect(classificationParser)
    @api.response(200, 'OK', classificationModel)
    @api.doc(responses={400:'X-Manual header not specified', 404:'Could not find classification with given ID'})
    def get(self, class_id):
        # Input Validation::
        manual = checkXManual(request)

        dao = getClassificationDAO(manual)

        result = dao.getClassification(class_id)
        if result is None:
            return {'message': 'Failed to locate classification with id {}'.format(class_id)}, 404

        return jsonify(result.toDict())


    @api.doc(description='Update information for the specified classification entry')
    @api.expect(classificationParser)
    @api.response(200, 'OK', classificationModel)
    @api.doc(responses={400:'X-Manual header not specified', 404:'Could not find classification with given ID'})
    def put(self, class_id):
        # Input Validation::
        manual = checkXManual(request)

        dao = getClassificationDAO(manual)

        result = dao.updateClassification(class_id, request.get_json())
        if result is None:
            return {'message': 'No image with id {} found with a classification to update or your input was invalid (or was there a server error?)'.format(class_id)}, 404
        else:
            return jsonify(result.toDict())
