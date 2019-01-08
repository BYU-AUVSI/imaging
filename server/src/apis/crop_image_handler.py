from flask import jsonify, make_response, send_file, abort, request
from flask_restplus import Namespace, Resource, fields
from dao.manual_cropped_dao import ManualCroppedDAO
from dao.model.manual_cropped import manual_cropped
from dao.model.point import point
from config import defaultConfigPath, defaultCroppedImgPath, allowedFileType
from werkzeug.utils import secure_filename
from werkzeug.datastructures import FileStorage
import os, time

api  = Namespace('image/crop', description="All cropped image calls route through here")

imageIDParser = api.parser()
imageIDParser.add_argument('X-Image-Id', location='headers', type=int, required=True, help='Specify the associated image id for this image.')
imageIDParser.add_argument('cropped_image', type=FileStorage, location='files', required=True, help='The cropped image file')

croppedImageModel = api.model('Crop Image Info', {
    'id': fields.Integer(description='Auto-generated id for the cropped image', example=1234),
    'image_id': fields.Integer(description='Id for the Raw image this crop came from. Image_id corresponds to the "id" column/value from a raw image / incoming_image table', example=12),
    'timestamp': fields.Float(description='Unix epoch UTC timestamp of when the image was submitted to the server (not particularly helpful)', example=1541063315.2),
    'cropped_path': fields.String(description='SERVER-SIDE absolute path to the cropped image this data corresponds to', example='/Users/len0rd/code/auvsi/imaging/server/src/images/9999/crop/051117-09_41_14_523.jpg'),
    'crop_coordinate_tl': fields.String(description='The top-left coordinate where the crop took place on the raw image. This coordinate corresponds to the top-left point of the bounding box used to define this cropped image. Coordinates are in pixels', example='(12345,12345)'),
    'crop_coordinate_br': fields.String(description='The bottom-right coordinate where the crop took place on the raw image. This coordinate corresponds to the bottom-right point of the bounding box used to define this cropped image. Coordinates are in pixels', example='(12999,66677)'),
    'crop_coordinate_tl.x': fields.Integer(description='The X component of the above top-left coordinate as an integer', example=12345),
    'crop_coordinate_tl.y': fields.Integer(description='The Y component of the above top-left coordinate as an integer', example=12345),
    'crop_coordinate_br.x': fields.Integer(description='The X component of the above bottom_right coordinate as an integer', example=12999),
    'crop_coordinate_br.y': fields.Integer(description='The Y component of the above bottom_right coordinate as an integer', example=66677)
})

@api.route('/')
class CroppedImageHandler(Resource):
    @api.doc(description='Gets the next un-tapped cropped image')
    @api.doc(responses={200:'OK', 404:'No available images found'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned. Not this is a unique from the base image_id')
    def get(self):
        # Get content:
        dao = ManualCroppedDAO(defaultConfigPath())
        image  = dao.getNextImage()

        # response validation
        if image is None:
            return {'message': 'Failed to locate untapped image'}, 404
        
        # success!
        return cropImageSender(image.id, image.cropped_path)
    
    @api.doc(description='Adds a new cropped image to the server')
    @api.expect(imageIDParser)
    @api.doc(responses={200:'OK', 400:'Improper image post'})
    @api.header('X-Crop-Id', 'Crop ID of the image if successfully inserted. This WILL be different from the Image-ID provided in the request')
    def post(self):
        # basic setup pieces for this taken from :
        # http://flask.pocoo.org/docs/1.0/patterns/fileuploads/
        # Input Validation:
        if 'cropped_image' not in request.files:
            abort(400, "Need to pass an image with the key 'cropped_image'")
        imageFile = request.files.get('cropped_image', '')
        
        cropped = manual_cropped()
        if 'X-Image-Id' in request.headers:
            cropped.image_id = request.headers.get('X-Image-Id')
        else:
            abort(400, "Need to specify header 'X-Image-Id'!")

        # make sure the filename wont make our computer explode:
        if imageFile.filename == '' or imageFile.filename == 'cropped_image':
            imageFile.filename = str(int(time.time())) + '.jpg'
        elif not allowedFileType(imageFile.filename):
            abort(400, "Filename invalid!")
        filename = secure_filename(imageFile.filename)
        
        # save image
        full_path = os.path.join(defaultCroppedImgPath(), filename)
        imageFile.save(full_path)

        if 'crop_coordinate_tl' in request.form:
            cropped.crop_coordinate_tl = point(ptStr=request.form['crop_coordinate_tl'])
        if 'crop_coordinate_br' in request.form:
            cropped.crop_coordinate_br = point(ptStr=request.form['crop_coordinate_br'])

        # add to db
        cropped.time_stamp = int(time.time())
        cropped.cropped_path = full_path

        dao = ManualCroppedDAO(defaultConfigPath())
        # resultingId is the manual_cropped.id value given to this image (abstracted from client)
        resultingId = dao.upsertCropped(cropped)
        
        if resultingId == -1:
            return {'message': 'Failed to insert image into manual_cropped! (If youre trying to update information on an image_id that already exists, you should use PUT)'}, 500
        
        # done!
        response = make_response(jsonify({'message': 'success!', 'id': resultingId}))
        response.headers['X-Crop-Id'] = resultingId
        return response


@api.route('/<int:crop_id>')
@api.doc(params={'crop_id': 'Cropped ID of the cropped image to retrieve'}, required=True)
class SpecificCroppedImageHandler(Resource):
    @api.doc(description='Attempts to retrieve the cropped image with the given id')
    @api.doc(responses={200:'OK', 404:'Cropped image with this id not found'})
    @api.header('X-Crop-Id', 'Crop Id of the image returned. Will match id parameter image was found')
    def get(self, crop_id):
        # Get content:
        dao = ManualCroppedDAO(defaultConfigPath())
        image = dao.getImage(crop_id)

        # response validation:
        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(crop_id)}, 404
        
        # success!
        return cropImageSender(image.id, image.cropped_path)


@api.route('/<int:crop_id>/info')
@api.doc(params={'crop_id': 'ID of the cropped image to update or get info on'}, required=True)
class SpecificCroppedImageInfoHandler(Resource):
    @api.doc(description='Get information about a cropped image from the database.')
    @api.response(200, 'OK', croppedImageModel)
    @api.doc(responses={404: 'Failed to locate the given crop_id in the table'})
    def get(self, crop_id):
        dao = ManualCroppedDAO(defaultConfigPath())
        image = dao.getImage(crop_id)

        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(crop_id)}, 404
        return jsonify(image.toJsonResponse())

    @api.doc(description='Update information on the specified cropped image')
    @api.response(200, 'OK', croppedImageModel)
    @api.doc(responses={400: 'Either: a) you didnt specify any values to update in the request body, b) you attempted to update image_id, or c) you attempted to update the id (aka crop_id)', 404: 'Failed to locate the given crop_id in the table'})
    def put(self, crop_id):
        content = request.get_json()
        if content is None:
            abort(400, 'Must specify values to update')
        if 'image_id' in content:
            abort(400, 'Updating image_id is forbidden!')
        if 'id' in content:
            abort(400, "Updating the crop_id (id) is forbidden")

        dao = ManualCroppedDAO(defaultConfigPath())
        result = dao.updateImage(crop_id, content)
        if result is None:
            return {'message': 'No image with id {} found to update (or was there a server error?)'.format(crop_id)}, 404
        else:
            return jsonify(result.toJsonResponse())


@api.route('/all')
class AllCroppedImagesHandler(Resource):
    @api.doc(description='Get info on all the cropped images currently in the queue')
    # support getting only tapped, and getting from list
    def get(self):
        dao = ManualCroppedDAO(defaultConfigPath())
        manualCroppedList = dao.getAll()

        if manualCroppedList == None or not manualCroppedList:
            return {'message': 'Cropped table is empty!'}, 404

        exportable = [ img.toJsonResponse() for img in manualCroppedList ]
        return jsonify(exportable)
        

def cropImageSender(id, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))
    response.headers['X-Crop-Id'] = id
    return response