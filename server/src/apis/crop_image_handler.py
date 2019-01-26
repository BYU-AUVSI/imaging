from flask import jsonify, make_response, send_file, abort, request
from flask_restplus import Namespace, Resource, fields, inputs
from dao.cropped_manual_dao import CroppedManualDAO
from dao.cropped_autonomous_dao import CroppedAutonomousDAO
from dao.model.cropped_manual import cropped_manual
from dao.model.cropped_autonomous import cropped_autonomous
from dao.incoming_image_dao import IncomingImageDAO
from dao.model.point import point
from config import defaultConfigPath, defaultCroppedImgPath, allowedFileType
from werkzeug.utils import secure_filename
from werkzeug.datastructures import FileStorage
from apis.helper_methods import checkXManual
import os, time

api  = Namespace('image/crop', description="All cropped image calls route through here")

xManParser = api.parser()
xManParser.add_argument('X-Manual', location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)') 

imageIDParser = api.parser()
imageIDParser.add_argument('X-Image-Id', location='headers', type=int, required=True, help='Specify the associated image id for this image.')
imageIDParser.add_argument('X-Manual',   location='headers', type=inputs.boolean, required=True, help='Specify whether this is a manual request (True) or autonomous (False)')
imageIDParser.add_argument('crop_coordinate_tl',   location='form', type=str, required=False, help='Optionally specify the top-left coordinate of the crop bounding box')
imageIDParser.add_argument('crop_coordinate_br',   location='form', type=str, required=False, help='Optionally specify the bottom-right coordinate of the crop bounding box')
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
    @api.expect(xManParser)
    def get(self):
        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        # Get content:
        dao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        image  = dao.getNextImage()

        # response validation
        if image is None:
            return {'message': 'Failed to locate untapped image'}, 404
        
        # success!
        return cropImageSender(image, image.cropped_path)
    
    @api.doc(description='Adds a new cropped image to the server')
    @api.expect(imageIDParser)
    @api.doc(responses={200:'OK', 400:'Improper image post'})
    @api.header('X-Crop-Id', 'Crop ID of the image if successfully inserted. This WILL be different from the Image-ID provided in the request')
    def post(self):
        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        # basic setup pieces for this taken from :
        # http://flask.pocoo.org/docs/1.0/patterns/fileuploads/
        # Input Validation:
        if 'cropped_image' not in request.files:
            abort(400, "Need to pass an image with the key 'cropped_image'")
        imageFile = request.files.get('cropped_image', '')
        
        # use the right model object depending on if this is a manual or autonomous request
        cropped = cropped_manual() if manual else cropped_autonomous()
        # confirm that we got an image_id in the headers
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
        cropped.cropped_path = full_path

        # check for the optional crop coordinate form data
        if 'crop_coordinate_tl' in request.form:
            cropped.crop_coordinate_tl = point(ptStr=request.form['crop_coordinate_tl'])
        if 'crop_coordinate_br' in request.form:
            cropped.crop_coordinate_br = point(ptStr=request.form['crop_coordinate_br'])

        # get the timestamp from incoming_image table to copy into this guy....
        # meh, but functional /shrug
        dao = IncomingImageDAO(defaultConfigPath())
        img = dao.getImage(cropped.image_id)
        if img is not None:
            cropped.time_stamp = img.time_stamp
        else:
            print("WARN:: failed to find incoming_image model at id {} X-Image-Id".format(cropped.image_id))
            cropped.time_stamp = int(time.time())

        # add to db
        dao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        # resultingId is the cropped_manual.id value given to this image (abstracted from client)
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
    @api.expect(xManParser)
    def get(self, crop_id):
        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        # Get content:
        dao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        image = dao.getImage(crop_id)

        # response validation:
        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(crop_id)}, 404
        
        # success!
        return cropImageSender(image, image.cropped_path)


@api.route('/<int:crop_id>/info')
@api.doc(params={'crop_id': 'ID of the cropped image to update or get info on'}, required=True)
class SpecificCroppedImageInfoHandler(Resource):
    @api.doc(description='Get information about a cropped image from the database.')
    @api.response(200, 'OK', croppedImageModel)
    @api.doc(responses={404: 'Failed to locate the given crop_id in the table'})
    @api.expect(xManParser)
    def get(self, crop_id):
        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        dao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        image = dao.getImage(crop_id)

        if image is None:
            return {'message': 'Failed to locate cropped id {}'.format(crop_id)}, 404
        return jsonify(image.toJsonResponse())

    @api.doc(description='Update information on the specified cropped image')
    @api.response(200, 'OK', croppedImageModel)
    @api.doc(responses={400: 'Either: a) you didnt specify any values to update in the request body, b) you attempted to update image_id, or c) you attempted to update the id (aka crop_id)', 404: 'Failed to locate the given crop_id in the table'})
    @api.expect(xManParser)
    def put(self, crop_id):

        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        content = request.get_json()
        if content is None:
            abort(400, 'Must specify values to update')
        if 'image_id' in content:
            abort(400, 'Updating image_id is forbidden!')
        if 'crop_id' in content:
            abort(400, "Updating the crop_id is forbidden")

        dao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        result = dao.updateImage(crop_id, content)
        if result is None:
            return {'message': 'No image with id {} found to update (or was there a server error?)'.format(crop_id)}, 404
        else:
            return jsonify(result.toJsonResponse())


@api.route('/all')
class AllCroppedImagesHandler(Resource):
    @api.doc(description='Get info on all the cropped images currently in the queue')
    # support getting only tapped, and getting from list
    @api.expect(xManParser)
    def get(self):

        # confirm that the X-Manual header was specified
        manual = checkXManual(request)

        dao = CroppedManualDAO(defaultConfigPath()) if manual else CroppedAutonomousDAO(defaultConfigPath())
        croppedList = dao.getAll()

        if croppedList == None or not croppedList:
            return {'message': 'Cropped table is empty!'}, 404

        exportable = [ img.toJsonResponse() for img in croppedList ]
        return jsonify(exportable)
        

def cropImageSender(cropImageModel, filename):
    response = make_response(send_file(filename, as_attachment=False, attachment_filename=filename, mimetype='image/jpeg'))
    response.headers['X-Crop-Id'] = cropImageModel.crop_id
    response.headers['X-Image_id'] = cropImageModel.image_id
    if hasattr(cropImageModel, 'time_stamp'):
        response.headers['X-time_stamp'] = cropImageModel.time_stamp
    return response