from flask import jsonify, request
from flask_restplus import Namespace, Resource, inputs
from dao.outgoing_manual_dao import OutgoingManualDAO
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_manual import outgoing_manual
from dao.model.outgoing_autonomous import outgoing_autonomous
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
    def get(self):
        manual = checkXManual(request)
        print(manual)
        dao = getClassificationDAO(manual)

        results = dao.getPendingTargets()
        print(results)
        if results is None or not results or not results[0]:
            return {'message': 'Could not locate any distinct targets'}, 404

        jsonible = []
        for target in results:
            jsonible.append([ classification.toDict(exclude=('id',)) for classification in target ])

        return jsonify(jsonible)