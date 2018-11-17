from flask import jsonify, make_response, send_file, abort, request
from flask_restplus import Namespace, Resource, fields
from dao.manual_cropped_dao import ManualCroppedDAO
from dao.model.manual_cropped import manual_cropped
from config import defaultSqlConfigPath, defaultCroppedImgPath, allowedFileType
from werkzeug.utils import secure_filename
import os, time

api  = Namespace('image/crop', description="All imaging related calls route through here")

newCroppedParser = api.parser()
newCroppedParser.add_argument('X-Raw-Id', location='headers', type=int, required=False, help='Specify the associated Raw id for this image. It is HIGHLY reccommended that you specify this header. Things may not work (geolocation) if you dont.')

@api.route('/')
class CroppedImageHandler(Resource):
    @api.doc(description='Gets the next un-tapped cropped image')
    @api.doc(responses={200:'OK', 404:'No available images found'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned.')
    def get(self):
        # Get content:
        dao = ManualCroppedDAO(defaultSqlConfigPath())
        image  = dao.getNextImage()

        # response validation
        if image is None:
            return {'message': 'Failed to locate untapped image'}, 404
        
        # success!
        return cropImageSender(image.id, image.cropped_path)
    
    @api.doc(description='Adds a new cropped image to the server')
    @api.expect(newCroppedParser)
    @api.doc(responses={200:'OK', 400:'Improper image post'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned.')
    def post(self):
        # basic setup pieces for this taken from :
        # http://flask.pocoo.org/docs/1.0/patterns/fileuploads/
        # Input Validation:
        if 'cropped_image' not in request.files:
            abort(400, "Need to pass an image with the key 'cropped_image'")
        imageFile = request.files.get('cropped_image', '')

        # make sure the filename wont make our computer explode:
        if imageFile.filename == '' or not allowedFileType(imageFile.filename):
            abort(400, "Filename invalid!")
        filename = secure_filename(imageFile.filename)
        
        # save it
        full_path = os.path.join(defaultCroppedImgPath(), filename)
        imageFile.save(full_path)

        # add to db
        cropped = manual_cropped()
        if 'X-Raw-Id' in request.headers:
            cropped.raw_id = request.headers.get('X-Raw-Id')
        cropped.time_stamp = int(time.time())
        cropped.cropped_path = full_path

        dao = ManualCroppedDAO(defaultSqlConfigPath())
        resultingId = dao.addImage(cropped)
        
        if resultingId == -1:
            return {'message': 'Failed to insert image into manual_cropped!'}, 500
        
        # done!
        response = make_response(jsonify({'message': 'success!', 'id': resultingId}))
        response.headers['X-Crop-Id'] = resultingId
        return response



@api.route('/<int:id>')
@api.doc(params={'id': 'ID of the cropped image to retrieve'}, required=True)
class SpecificCroppedImageHandler(Resource):
    @api.doc(description='Attempts to retrieve the cropped image with the given id')
    @api.doc(responses={200:'OK', 404:'Cropped image with this id not found'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned. Will match id parameter image was found')
    def get(self, id):
        # Get content:
        dao = ManualCroppedDAO(defaultSqlConfigPath())
        image = dao.getImage(id)

        # response validation:
        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(id)}, 404
        
        # success!
        return cropImageSender(image.id, image.cropped_path)


@api.route('/<int:id>/info')
@api.doc(params={'id': 'ID of the cropped image to update or get info on'}, required=True)
class SpecificCroppedImageInfoHandler(Resource):
    @api.doc(description='Get information about a cropped image from the database.')
    def get(self, id):
        dao = ManualCroppedDAO(defaultSqlConfigPath())
        image = dao.getImage(id)

        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(id)}, 404
        return jsonify(image.toJsonResponse())

    @api.doc(description='Update information on the specified cropped image')
    @api.doc()
    def put(self, id):
        content = request.get_json()
        if content is None:
            abort(400, 'Must specify values to update')

        dao = ManualCroppedDAO(defaultSqlConfigPath())
        result = dao.updateImage(id, content)
        if result == -1:
            return {'message': 'No image with id {} found to update'.format(id)}, 404
        else:
            return jsonify(result.toJsonResponse())


def cropImageSender(id, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))
    response.headers['X-Crop-Id'] = id
    return response