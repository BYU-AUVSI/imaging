from config import defaultConfigPath
from dao.incoming_state_dao import IncomingStateDAO
from flask import jsonify
from flask_restplus import Namespace, Resource, fields

api = Namespace('state', description="Retrieve state information")

# for documentation purposes, describes what's returned by the handlers for Swagger
stateModel = api.model('State', {
    'id': fields.Integer(description='Auto-generated id for a state measurement', example=1234),
    'timestamp': fields.Float(description='ROS unix epoch UTC timestamp for the state measurement', example=1541063315.2),
    'roll': fields.Float(description='Measured roll angle relative to NED in radians (TODO: verify this is correct)', example=0.012203852),
    'pitch': fields.Float(description='Measured pitch angle relative to NED in radians (TODO: verify)', example=0.09302688),
    'yaw': fields.Float(description='Measured yaw angle relative to NED in radians (TODO: verify)', example=-0.22642705)
})

@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the state measurement to retrieve'}, required = True)
class StateIdHandler(Resource):
    @api.doc(description='Get the state measurement with the given id')
    @api.response(200, 'OK', stateModel)
    @api.doc(responses={404:'Id not found'})
    def get(self, id):
        dao = IncomingStateDAO(defaultConfigPath())
        state = dao.getStateById(id)

        if state is None:
            return {'message': 'Failed to locate state entry with id {}'.format(id)}, 404
        
        return jsonify(state.toDict())

@api.route('/ts/<float:ts>')
@api.doc(params={'ts': 'Unix/Epoch UTC timestamp to query database for'}, required=True)
class StateTsHandler(Resource):
    @api.doc(description='Get the state measurement in the database closest to the provided time_stamp')
    @api.response(200, 'OK', stateModel)
    @api.doc(responses={404:'No state measurement found'})
    def get(self, ts):
        dao = IncomingStateDAO(defaultConfigPath())
        state = dao.getStateByClosestTS(ts)

        if state is None:
            return {'message': 'Failed to get closest state measurement by time_stamp. Either: a) the table is empty or b) the time_stampe is < all timestamps in the table'}, 404

        return jsonify(state.toDict())