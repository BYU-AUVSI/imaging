from flask_restplus import Namespace, Resource, inputs

api = Namespace('image/class', description='Imaging classification calls route through here')

classificationParser = api.parser()
classificationParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True)

classificationPostParser = api.parser()
classificationPostParser.add_argument('X-Crop-Id', location='headers', type=int, required=True)

@api.route('/')
class ClassificationHandler(Resource):
    
    @api.doc(description='MANUAL ONLY:: Gets the oldest non-completed classification')
    def get(self, id):
        return 404

    @api.doc(description='Create a new classification entry. NOTE: if an entry already exists for the given Crop-Id (specified in the header), then the entry will be updated.')
    @api.expect(classificationPostParser)
    def post(self):
        return 404


@api.route('/all')
class AllClassificationsHandler(Resource):

    @api.doc(description='Get all the classifications currently waiting for the corresponding process type (Manual or Autonomous)')
    @api.expect(classificationParser)
    def get(self):
        return 404


@api.route('/<int:image_id>')
@api.doc(params={'image_id': 'Classification ID of the classification entry to update or get info on'}, required=True)
class SpecificClassificationHandler(Resource):

    @api.doc(description='Update information for the specified classification entry')
    @api.expect(classificationParser)
    def put(self, image_id):
        return 404