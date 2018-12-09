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

classification = api.model('Classification', {
    'type': fields.String(required=False, description='Classification type(standard, off_axis, or emergent)'),
    'latitude': fields.Float(required=False, description='Latitude coordinate of object'),
    'longitude': fields.Float(required=False, description='longitude coordinate of object'),
    'orientation': fields.String(required=False, description='Describes the heading/orientation of the letter(N, NE, E, etc...)'),
    'shape': fields.String(required=False, description='The shape of the object for standard/off_axis types(circle, triangle, square, etc...)'),
    'background_color': fields.String(required=False, description='Color of the background the letter is on for standard/off_axis types(white, black, orange, etc...)'),
    'alphanumeric': fields.String(required=False, description='The letter within the object for standard/off_axis types(A, B, C, etc...)'),
    'alphanumeric_color': fields.String(required=False, description='Color of the letter for standard/off_axis types(white, black, orange, etc...)'),
    'description': fields.String(required=False, description='For the emergent type, description of what it is'),
    'submitted': fields.Boolean(required=False, description='Whether or not this particular classification has been submitted to the judges over interop(defaults to false)')
})

@api.route('/all')
class AllClassificationsHandler(Resource):

    @api.doc(description='Get all the completed classifications for the corresponding process type (Manual or Autonomous)')
    @api.expect(classificationParser)
    def get(self):
        # Input Validation::
        manual = checkXManual(request)

        outgoingList = []
        dao = getClassificationDAO(manual)
        outgoingList = dao.getAll()

        if not outgoingList:
            return {'message': 'Outgoing table is empty!'}, 404

        exportable = [ classification.toDict(exclude=('id',)) for classification in outgoingList ]
        return jsonify(exportable)


@api.route('/<int:image_id>')
@api.doc(params={'image_id': 'Image ID of the classification entry to update or get info on'}, required=True)
class SpecificClassificationHandler(Resource):

    @api.doc(description='Get the classification for the given image id')
    @api.expect(classificationParser)
    def get(self, image_id):
        # Input Validation::
        manual = checkXManual(request)

        dao = getClassificationDAO(manual)

        result = dao.getClassification(image_id)
        if result is None:
            return {'message': 'Failed to locate classification with id {}'.format(image_id)}, 404

        return jsonify(result.toDict(exclude=('id',)))

    @api.doc(description='Create a new classification entry. NOTE: if an entry already exists for the given Image-Id (specified in the header), then that entry will be updated with this information instead of inserted.')
    @api.expect(classificationParser)
    @api.expect(classification)
    def post(self, image_id):
        # Input Validation::
        manual = checkXManual(request)

        dao = getClassificationDAO(manual)
        if manual:
            outgoingIn = outgoing_manual(json=api.payload)
        else:
            outgoingIn = outgoing_autonomous(json=api.payload)
        outgoingIn.image_id = image_id
        
        resultingId = dao.upsertClassification(outgoingIn)

        if resultingId == -1:
            return {'message': 'Failed to insert classification into outgoing table'}, 500

        response = make_response(jsonify({'message': 'success!', 'id': image_id}))
        return response


    @api.doc(description='Update information for the specified classification entry')
    @api.expect(classificationParser)
    @api.expect(classification)
    def put(self, image_id):
        # Input Validation::
        manual = checkXManual(request)

        dao = getClassificationDAO(manual)

        result = dao.updateClassificationByUID(image_id, api.payload)
        if result is None:
            return {'message': 'No image with id {} found with a classification to update or your input was invalid (or was there a server error?)'.format(image_id)}, 404
        else:
            return jsonify(result.toDict())
