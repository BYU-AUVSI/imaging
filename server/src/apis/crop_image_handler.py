from flask import jsonify, make_response, send_file
from flask_restplus import Namespace, Resource, fields
from dao.manual_cropped_dao import ManualCroppedDAO
from config import defaultSqlConfigPath

api  = Namespace('image/crop', description="All imaging related calls route through here")

@api.route('/')
class CroppedImageHandler(Resource):
    @api.doc('Gets the next un-tapped cropped image')
    @api.doc(responses={200:'OK', 404:'No available images found'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned.')
    def get(self):
        return None
    
    @api.doc('Adds a new cropped image to the server')
    def post(self):
        return "success!"

@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the cropped image to retrieve'}, required=True)
class SpecificCroppedImageHandler(Resource):
    @api.doc('Attempts to retrieve the cropped image with the given id')
    @api.doc(responses={200:'OK', 404:'Cropped image with this id not found'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned. Will match id parameter image was found')
    def get(self, id):
        dao = ManualCroppedDAO(defaultSqlConfigPath())
        image = dao.getImage(id)

        # response validation:
        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(id)}, 404
        
        # success!
        return cropImageSender(image.id, image.cropped_path)


@api.route('/<int:id>/info')
@api.doc(params={'id': 'ID of the cropped image to update'}, required=True)
class SpecificCroppedImageInfoHandler(Resource):
    @api.doc('Put information about a cropped image into the database. Normally used to update cropped coordinates')
    def put(self, id):
        return None


def cropImageSender(id, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))
    response.headers['X-Crop-Id'] = id
    return response