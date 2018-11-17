from config import defaultSqlConfigPath
from dao.incoming_gps_dao import IncomingGpsDAO
from flask import jsonify
from flask_restplus import Namespace, Resource


api = Namespace('gps', description='Retrieve gps information')

@api.route('/<int:id>')
class GpsIdHandler(Resource):
    def get(self, id):
        dao = IncomingGpsDAO(defaultSqlConfigPath())
        gps = dao.getGpsById(id)

        # response validation
        if gps is None:
            return {'message': 'Failed to locate gps id {}'.format(id)}, 404
        
        # success!
        return jsonify(gps.toDict())

@api.route('/ts/<float:ts>')
class GpsTsHandler(Resource):
    def get(self, ts):
        dao = IncomingGpsDAO(defaultSqlConfigPath())
        gps = dao.getGpsByClosestTS(ts)

        # response validation:
        if gps is None:
            return {'message': 'Failed to get closest gps by time_stamp. Is the table empty?'}

        # success!
        return jsonify(gps.toDict())