from flask import send_file, make_response, Response, request, abort
import os
from flask_restplus import Namespace, Resource, inputs
from dao.incoming_image_dao import IncomingImageDAO
from config import defaultSqlConfigPath

api  = Namespace('image/raw', description="All imaging related calls route through here")

rawParser = api.parser()
rawParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)') 

@api.route('/')
@api.expect(rawParser)
class RawImageHandler(Resource):
    @api.doc('Gets the next un-tapped raw image')
    @api.doc(responses={200:'OK', 404:'No image found'})
    @api.header('X-Raw-Id', 'Raw Id of the image returned. Will match id parameter if one was specified')
    def get(self):
        # Input Validation::
        if 'X-Manual' not in request.headers:
            abort(400, 'Need to specify X-Manual header!')
        manual = request.headers.get('X-Manual')
        try:
            manual = bool(manual)
        except:
            abort(400, 'Failed to interpret X-Manual header as boolean!')

        # Get Content
        dao = IncomingImageDAO(defaultSqlConfigPath())
        image = dao.getNextImage(manual)

        # response validation
        if image is None:
            return {'message': 'Failed to locate unclaimed image'}, 404
        
        return rawImageSender(image.id, image.image_path)
        
@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the raw image to retrieve'}, required=True)
class SpecificRawImageHandler(Resource):
    @api.doc('Attempts to retrieve a raw image with the given id.')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    @api.header('X-Raw-Id', 'Raw Id of the image returned. Will match id parameter if one was specified')
    def get(self, id):
        dao = IncomingImageDAO(defaultSqlConfigPath())
        image  = dao.getImage(id)
        if image is None:
            return {'message': 'Failed to locate id {}'.format(id)}, 404

        # otherwise lets send the image::
        return rawImageSender(image.id, image.image_path)
        
def rawImageSender(id, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))        
    response.headers['X-Raw-Id'] = id
    return response
