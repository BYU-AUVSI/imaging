from flask import Flask
from flask_restplus import Api, Resource
from api.image_handler import RawImageHandler, CroppedImageHandler

app = Flask(__name__)
api = Api(app, title="BYU AUVSI Imaging", description="REST API for BYU AUVSI Imaging")
ns  = api.namespace('image', description="All imaging related calls route through here")

# RAW Endpoints:
ns.add_resource(RawImageHandler, '/raw') # get the next un-tapped image for manual or auto
ns.add_resource(RawImageHandler, '/raw/<int:id>') # get specific raw image by id


# CROPPED Endpoints:
ns.add_resource(CroppedImageHandler, '/crop') # get the next un-tapped cropped image for manual or auto
ns.add_resource(CroppedImageHandler, '/crop/<int:id>') # get specific cropped image by id

if __name__ == '__main__':
    app.run(debug=True)