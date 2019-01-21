from flask import send_file, make_response, request, abort, jsonify
import os
import time
from flask_restplus import Namespace, Resource, inputs, fields
from dao.incoming_image_dao import IncomingImageDAO
from config import defaultConfigPath
from apis.helper_methods import checkXManual

api  = Namespace('image/raw', description="All raw image functions route through here")

rawParser = api.parser()
rawParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)') 

rawImageModel = api.model('Raw Image Info', {
    'id': fields.Integer(description='Auto-generated id for the image', example=1234),
    'timestamp': fields.Float(description='ROS unix epoch UTC timestamp for the image', example=1541063315.2),
    'image_path': fields.String(description='SERVER-SIDE absolute path to the image this data represents', example='/Users/len0rd/code/auvsi/ws_server_test/src/imaging/server/src/../images/1546372042/raw/1541519087.0.jpg'),
    'manual_tap': fields.Boolean(description='Whether this image has been requested and sent to a manual imaging client yet. Defaults to False'),
    'autonomous_tap': fields.Boolean(description='Whether this image has been requested and sent to an autonomous imaging client yet. Defaults to False')
})

@api.route('/')
@api.expect(rawParser)
class RawImageHandler(Resource):
    @api.doc(description='Gets the next un-tapped raw image')
    @api.doc(responses={200:'OK', 404:'No image found'})
    @api.header('X-Image-Id', 'Raw Id of the image returned.')
    def get(self):
        # startTime = time.time()
        # Input Validation::
        manual = checkXManual(request)

        # Get Content
        dao = IncomingImageDAO(defaultConfigPath())
        image = dao.getNextImage(manual)

        # response validation
        if image is None:
            return {'message': 'Failed to locate untapped image'}, 404
        
        # print("Request fulfillment: {}".format(time.time()-startTime))
        return rawImageSender(image.image_id, image.image_path)
        
@api.route('/<int:image_id>')
@api.doc(params={'image_id': 'ID of the raw image to retrieve'}, required=True)
class SpecificRawImageHandler(Resource):
    @api.doc(description='Attempts to retrieve a raw image with the given id.')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    @api.header('X-Image-Id', 'Id of the image returned. Will match id parameter if image was found')
    def get(self, image_id):
        dao = IncomingImageDAO(defaultConfigPath())
        image  = dao.getImage(image_id)
        if image is None:
            return {'message': 'Failed to locate raw id {}'.format(image_id)}, 404

        # otherwise lets send the image::
        return rawImageSender(image.image_id, image.image_path)
        

@api.route('/<int:image_id>/info')
@api.doc(params={'image_id': 'ID of the image to update or get the raw info on'}, required=True)
class SpecificRawImageInfoHandler(Resource):
    @api.doc(description='Get information about a raw image from the database')
    @api.response(200, 'OK', rawImageModel)
    @api.doc(responses={404:'Id not found'})
    def get(self, image_id):
        dao = IncomingImageDAO(defaultConfigPath())
        image = dao.getImage(image_id)

        if image is None:
            return {'message': 'Failed to locate raw id {}'.format(image_id)}, 404
        return jsonify(image.toDict())


def rawImageSender(id, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))
    response.headers['X-Image-Id'] = id
    return response
