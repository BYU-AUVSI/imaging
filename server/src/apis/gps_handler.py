from config import defaultSqlConfigPath
from dao.incoming_gps_dao import IncomingGpsDAO
from flask import jsonify
from flask_restplus import Namespace, Resource

api = Namespace('gps', description='Retrieve gps information')

@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the gps measurement to retrieve'}, required=True)
class GpsIdHandler(Resource):
    @api.doc(description='Get the gps measurement with the given id')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    def get(self, id):
        dao = IncomingGpsDAO(defaultSqlConfigPath())
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
    @api.doc(responses={200:'OK', 404:'No Gps measurement found'})
    def get(self, ts):
        dao = IncomingGpsDAO(defaultSqlConfigPath())
        gps = dao.getGpsByClosestTS(ts)

        # response validation:
        if gps is None:
            return {'message': 'Failed to get closest gps by time_stamp. Is the table empty?'}, 404
        # success!
        return jsonify(gps.toDict())