from flask import jsonify, send_file
import os
from flask_restplus import Namespace, Resource, inputs

api  = Namespace('image/raw', description="All imaging related calls route through here")

rawParser = api.parser()
rawParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)') 

@api.route('/')
@api.expect(rawParser)
@api.doc()
class RawImageHandler(Resource):
    @api.doc('Gets the next un-tapped raw image')
    @api.doc(responses={200:'OK', 404:'No image found'})
    @api.header('X-Raw-Id', 'Raw Id of the image returned. Will match id parameter if one was specified')
    def get(self):
        # if no idea was sent then we need to get the next non-tapped image for this type:
        filename = os.path.dirname(os.path.realpath(__file__)) + '/../../images/raw/frame0571.jpg'
        return send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg')

@api.route('/<int:id>')
@api.expect(rawParser)
@api.doc(params={'id': 'ID of the raw image to retrieve'}, required=True)
class SpecificRawImageHandler(Resource):
    @api.doc('Attempts to retrieve a raw image with the given id.')
    @api.doc(responses={200:'OK', 404:'Id not found'})
    def get(self, id):
        return jsonify({'yousent': id})