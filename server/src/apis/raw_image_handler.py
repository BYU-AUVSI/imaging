from flask import jsonify, send_file
from flask_restplus import Namespace, Resource, fields

DEFAULT_ID = -1 # default to this if no id is specified in the request

api  = Namespace('image/raw', description="All imaging related calls route through here")

@api.route('/')
@api.route('/<int:id>')
class RawImageHandler(Resource):
    def get(self, id=DEFAULT_ID):
        if id == DEFAULT_ID:
            return send_file(filename, as_attachment=True, attachment_filename=filename, mimetype='image/jpeg')
        else:
            return jsonify({'yousent': id})
