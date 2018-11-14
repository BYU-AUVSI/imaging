from flask import jsonify
from flask_restplus import Resource

DEFAULT_ID = -1 # default to this if no id is specified in the request

class RawImageHandler(Resource):
    def get(self, id=DEFAULT_ID):
        if id == DEFAULT_ID:
            return jsonify({"yousent": "nothin"})
        else:
            return jsonify({'yousent': id})

class CroppedImageHandler(Resource):
    def get(self, id=DEFAULT_ID):
        if id == DEFAULT_ID:
            return jsonify({"yousent": "nothin"})
        else:
            return jsonify({'yousent': id})

    def post(self, id=DEFAULT_ID):
        return "success!"