from flask import jsonify
from flask_restplus import Namespace, Resource, fields

DEFAULT_ID = -1 # default to this if no id is specified in the request

api  = Namespace('image/crop', description="All imaging related calls route through here")

@api.route('/')
@api.route('/<int:id>')
class CroppedImageHandler(Resource):
    def get(self, id=DEFAULT_ID):
        if id == DEFAULT_ID:
            return jsonify({"yousent": "nothin"})
        else:
            return jsonify({'yousent': id})

    def post(self, id=DEFAULT_ID):
        return "success!"
