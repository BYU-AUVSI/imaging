from config import defaultSqlConfigPath
from dao.incoming_state_dao import IncomingStateDAO
from flask import jsonify
from flask_restplus import Namespace, Resource

api = Namespace('state', description="Retrieve state information")

@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the state measurement to retrieve'}, required = True)
class StateIdHandler(Resource):
    @api.doc(description='Get the state measurement with the given id')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    def get(self, id):
        dao = IncomingStateDAO(defaultSqlConfigPath())
        state = dao.getStateById(id)

        if state is None:
            return {'message': 'Failed to locate state entry with id {}'.format(id)}, 404
        
        return jsonify(state.toDict())

@api.route('/ts/<float:ts>')
@api.doc(params={'ts': 'Unix/Epoch UTC timestamp to query database for'}, required=True)
class StateTsHandler(Resource):
    @api.doc(description='Get the state measurement in the database closest to the provided time_stamp')
    @api.doc(responses={200:'OK', 404:'No state measurement found'})
    def get(self, ts):
        dao = IncomingStateDAO(defaultSqlConfigPath())
        state = dao.getStateByClosestTS(ts)

        if state is None:
            return {'message': 'Failed to get closest state measurement by time_stamp. Either: a) the table is empty or b) the time_stampe is < all timestamps in the table'}

        return jsonify(state.toDict())