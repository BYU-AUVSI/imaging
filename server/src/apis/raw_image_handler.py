from flask import send_file, make_response, request, abort, jsonify
import os
from flask_restplus import Namespace, Resource, inputs
from dao.incoming_image_dao import IncomingImageDAO
from config import defaultSqlConfigPath

api  = Namespace('image/raw', description="All raw image functions route through here")

rawParser = api.parser()
rawParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)') 

@api.route('/')
@api.expect(rawParser)
class RawImageHandler(Resource):
    @api.doc(description='Gets the next un-tapped raw image')
    @api.doc(responses={200:'OK', 404:'No image found'})
    @api.header('X-Image-Id', 'Raw Id of the image returned.')
    def get(self):
        # Input Validation::
        if 'X-Manual' not in request.headers:
            abort(400, 'Need to specify X-Manual header!')
        manual = request.headers.get('X-Manual')
        try:
            manual = manual.lower() == 'true'
        except:
            abort(400, 'Failed to interpret X-Manual header as boolean!')

        # Get Content
        dao = IncomingImageDAO(defaultSqlConfigPath())
        image = dao.getNextImage(manual)

        # response validation
        if image is None:
            return {'message': 'Failed to locate untapped image'}, 404
        
        return rawImageSender(image.image_id, image.image_path)
        
@api.route('/<int:image_id>')
@api.doc(params={'image_id': 'ID of the raw image to retrieve'}, required=True)
class SpecificRawImageHandler(Resource):
    @api.doc(description='Attempts to retrieve a raw image with the given id.')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    @api.header('X-Image-Id', 'Id of the image returned. Will match id parameter if image was found')
    def get(self, image_id):
        dao = IncomingImageDAO(defaultSqlConfigPath())
        image  = dao.getImage(image_id)
        if image is None:
            return {'message': 'Failed to locate raw id {}'.format(id)}, 404

        # otherwise lets send the image::
        return rawImageSender(image.image_id, image.image_path)
        

@api.route('/<int:image_id>/info')
@api.doc(params={'image_id': 'ID of the image to update or get the raw info on'}, required=True)
class SpecificRawImageInfoHandler(Resource):
    @api.doc(description='Get information about a raw image from the database')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    def get(self, image_id):
        dao = IncomingImageDAO(defaultSqlConfigPath())
        image = dao.getImage(image_id)

        if image is None:
            return {'message': 'Failed to locate raw id {}'.format(id)}, 404
        return jsonify(image.toDict())


def rawImageSender(id, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))
    response.headers['X-Image-Id'] = id
    return response
