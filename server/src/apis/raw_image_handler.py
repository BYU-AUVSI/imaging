from flask import jsonify, send_file, Response
import os
from flask_restplus import Namespace, Resource, fields

DEFAULT_ID = -1 # default to this if no id is specified in the request

api  = Namespace('image/raw', description="All imaging related calls route through here")

rawParser = api.parser()
rawParser.add_argument('X-Manual', location='headers') 

@api.route('/')
@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the raw image to retrieve'})
@api.expect(rawParser)
class RawImageHandler(Resource):
    @api.doc('If an ID is provided, retrieve that raw image, otherwise get the next un-tapped raw image')
    @api.doc(responses={200:'OK', 404:'No image found'})
    @api.header('X-Raw-Id', 'Raw Id of the image returned. Will match id parameter if one was specified')
    def get(self, id=DEFAULT_ID):
        # if no idea was sent then we need to get the next non-tapped image for this type:
        if id == DEFAULT_ID:
            filename = os.path.dirname(os.path.realpath(__file__)) + '/../../images/raw/frame0571.jpg'
            return send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg')
        else:
            return jsonify({'yousent': id})
