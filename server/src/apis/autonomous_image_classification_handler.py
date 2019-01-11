import os, time
from werkzeug.utils import secure_filename
from werkzeug.datastructures import FileStorage
from config import defaultConfigPath, defaultCroppedImgPath, allowedFileType
from flask import request, jsonify, abort, make_response
from flask_restplus import Namespace, Resource, fields
from dao.outgoing_autonomous_dao import OutgoingAutonomousDAO
from dao.model.outgoing_autonomous import outgoing_autonomous
from apis.helper_methods import checkXManual, getClassificationDAO

api = Namespace('image/class/autonomous', description="Image classification calls for autonomous clients route through here")

imageIDParser = api.parser()
imageIDParser.add_argument('X-Image-Id', location='headers', type=int, required=True, help='The original raw image_id this classification comes from')
imageIDParser.add_argument('cropped_image', type=FileStorage, location='files', required=True, help='The cropped image file')

# for documentation purposes. Defines the response for some of the methods below
classificationModel = api.model('Autonomous Classification', {
    'target': fields.Integer(required=False, description='which target number this is. This field is automatically managed by the server. It groups classifications together based on alphanumeric, shape and type', example=1),
    'image_id': fields.Integer(reqired=True, description='Id of the raw image this classification originally comes from', example=123),
    'crop_path': fields.String(reqired=True, description='SERVER-SIDE absolute path to the cropped image created by an autonomous client', example='/Users/len0rd/code/auvsi/imaging/server/src/images/9999/crop/autonomous/154423232.jpg'),
    'type': fields.String(required=False, description='Classification type(standard, off_axis, or emergent)', example="standard"),
    'latitude': fields.Float(required=False, description='Latitude coordinate of object', example=40.246354),
    'longitude': fields.Float(required=False, description='longitude coordinate of object', example=-111.647553),
    'orientation': fields.String(required=False, description='Describes the heading/orientation of the letter(N, NE, E, etc...)', example='N'),
    'shape': fields.String(required=False, description='The shape of the object for standard/off_axis types(circle, triangle, square, etc...)', example='circle'),
    'background_color': fields.String(required=False, description='Color of the background the letter is on for standard/off_axis types(white, black, orange, etc...)', example='orange'),
    'alphanumeric': fields.String(required=False, description='The letter within the object for standard/off_axis types(A, B, C, etc...)', example='A'),
    'alphanumeric_color': fields.String(required=False, description='Color of the letter for standard/off_axis types(white, black, orange, etc...)', example='black'),
    'description': fields.String(required=False, description='For the emergent type, description of what it is'),
    'submitted': fields.String(required=False, description='Whether or not this particular classification has been submitted to the judges. Defaults to "unsubmitted". "submitted" indicates that this classification has been submitted to the judges. "inherited_submission" indicates that another classification for the same target (ie: another image of the same target) has already been submitted, and its therefore unnecessary to submit this one.', example='unsubmitted')
})

@api.route("/all")
class AllAutonomousClassificationsHandler(Resource):
    @api.doc(description='Get all the manual classifications currently on the server')
    @api.response(200, 'OK', [classificationModel])
    @api.doc(responses={404:'No classifications (table empty)'})
    def get(self):
        dao = OutgoingAutonomousDAO(defaultConfigPath())
        outgoingList = dao.getAll()

        if not outgoingList:
            return {'message', 'No classifications found (table empty?)'}, 404
        
        exportable = [ classification.toDict() for classification in outgoingList ]
        return exportable


@api.route("/")
class AutonomousClassificationImageHandler(Resource):
    @api.doc(description="Add a new classification to the server. Requires an image in the payload. You can also include classification details as request headers if desired, so that an additional put request is not required")
    @api.doc(responses={200:'OK', 400: 'Improper image post (are you correctly adding an image to the request payload?)', 500: 'Something failed server-side'})
    @api.expect(imageIDParser)
    @api.header('X-Class-Id', 'Classification ID of the image if successfully inserted')
    def post(self):
        if 'cropped_image' not in request.files:
            abort(400, 'Need to pass an image file in the request with the key "cropped_image"')
        if 'X-Image-Id' not in request.headers:
            abort(400, 'Need to specify the original image id in the request header as X-Image_Id')    

        imageFile = request.files.get('cropped_image', '')
        
        # make sure the filename wont make our computer explode:
        if imageFile.filename == '' or imageFile.filename == 'cropped_image':
            imageFile.filename = str(int(time.time())) + '.jpg'
        elif not allowedFileType(imageFile.filename):
            abort(400, "Filename invalid!")
        filename = secure_filename(imageFile.filename)

        fullPath = os.path.join(defaultCroppedImgPath(), 'autonomous', filename)
        imageFile.save(fullPath)

        model = outgoing_autonomous()
        model.image_id  = request.headers.get('X-Image-Id')
        model.crop_path = fullPath

        # add any additional attributes transported in the header
        # to our model
        for attr in model.allProps():
            if attr in request.headers:
                setattr(model, attr, request.headers.get(attr))

        dao = OutgoingAutonomousDAO(defaultConfigPath())
        resultingId = dao.addClassification(model)  

        if resultingId == -1:
            return {'message': 'Failed to insert classification into outgoing_autonomous'}, 500
        
        # done!
        response = make_response(jsonify({'message': 'success!', 'id': resultingId}))
        response.headers['X-Class-Id'] = resultingId
        return response


@api.route('/<int:class_id>/info')
@api.doc(params={'class_id': 'Classification ID of the classification entry to update or get info on'}, required=True)
class AutonomousSpecificClassificationHandler(Resource):

    @api.doc(description='Get the classification for the given id')
    @api.response(200, 'OK', classificationModel)
    @api.doc(responses={404:'Could not find classification with given ID'})
    def get(self, class_id):

        dao = OutgoingAutonomousDAO(defaultConfigPath())

        result = dao.getClassification(class_id)
        if result is None:
            return {'message': 'Failed to locate classification with id {}'.format(class_id)}, 404

        return jsonify(result.toDict())

    @api.doc(description='Update information for the specified classification entry')
    @api.response(200, 'OK', classificationModel)
    @api.doc(responses={400:'X-Manual header not specified', 404:'Could not find classification with given ID'})
    def put(self, class_id):

        dao = OutgoingAutonomousDAO(defaultConfigPath())

        result = dao.updateClassification(class_id, request.get_json())
        if result is None:
            return {'message': 'No image with id {} found with a classification to update or your input was invalid (or was there a server error?)'.format(class_id)}, 404
        else:
            return jsonify(result.toDict())

    @api.doc(description='Delete the given classification from the ougoing table')
    @api.doc(responses={404:'Could not find classification with given ID', 500: 'The delete didnt work for some reason'})
    def delete(self, class_id):
        dao = OutgoingAutonomousDAO(defaultConfigPath())

        if dao.getClassification(class_id) is None:
            return {'message': 'Couldnt find classification with id {}'.format(class_id)}, 404

        result = dao.removeClassification(class_id)
        if not result:
            return {'message': 'Something went wrong while trying to delete id {} (was it delete by someone else first??)'.format(class_id)}, 500
        
        return {'message': 'success!', 'id': class_id}