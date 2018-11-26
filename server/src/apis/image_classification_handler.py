from flask_restplus import Namespace, Resource, inputs

api = Namespace('image/class', description='Imaging classification calls route through here')

classificationParser = api.parser()
classificationParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True)

@api.route('/all')
class AllClassificationsHandler(Resource):

    @api.doc(description='Get all the completed classifications for the corresponding process type (Manual or Autonomous)')
    @api.expect(classificationParser)
    def get(self):
        return 404


@api.route('/<int:image_id>')
@api.doc(params={'image_id': 'Image ID of the classification entry to update or get info on'}, required=True)
class SpecificClassificationHandler(Resource):

    @api.doc(description='Create a new classification entry. NOTE: if an entry already exists for the given Image-Id (specified in the header), then that entry will be updated with this information instead of inserted.')
    @api.expect(classificationParser)
    def post(self, id):
        return 404


    @api.doc(description='Update information for the specified classification entry')
    @api.expect(classificationParser)
    def put(self, image_id):
        return 404