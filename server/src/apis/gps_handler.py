from config import defaultConfigPath
from dao.incoming_gps_dao import IncomingGpsDAO
from flask import jsonify
from flask_restplus import Namespace, Resource, fields

api = Namespace('gps', description='Retrieve gps information')

# for documentation purposes. describes response to expect
gpsModel = api.model('Gps', {
    'id': fields.Integer(description='Auto-generated id for a gps measurement', example=1234),
    'timestamp': fields.Float(description='ROS unix epoch UTC timestamp for the gps measurement', example=1541063315.2),
    'latitude': fields.Float(description='GPS latitude measurement at timestamp', example=40.246354),
    'longitude': fields.Float(description='GPS longitude measurement at timestamp', example=-111.647553),
    'altitude': fields.Float(description='GPS altitude measurement at timestamp (in Ft)', example=1356.57)
})

@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the gps measurement to retrieve'}, required=True)
class GpsIdHandler(Resource):
    @api.doc(description='Get the gps measurement with the given id')
    @api.response(200, 'OK', gpsModel)
    @api.doc(responses={404:'Id not found'})
    def get(self, id):
        dao = IncomingGpsDAO(defaultConfigPath())
        gps = dao.getGpsById(id)

        # response validation
        if gps is None:
            return {'message': 'Failed to locate gps id {}'.format(id)}, 404
        
        # success!
        return jsonify(gps.toDict())

@api.route('/ts/<float:ts>')
@api.doc(params={'ts': 'Unix/Epoch UTC timestamp to query database for'}, required=True)
class GpsTsHandler(Resource):
    @api.doc(description='Get the gps measurement in the database closest to the provided time_stamp')
    @api.response(200, 'OK', gpsModel)
    @api.doc(responses={404:'No Gps measurement found'})
    def get(self, ts):
        dao = IncomingGpsDAO(defaultConfigPath())
        gps = dao.getGpsByClosestTS(ts)

        # response validation:
        if gps is None:
            return {'message': 'Failed to get closest gps by time_stamp. Is the table empty?'}, 404
        # success!
        return jsonify(gps.toDict())