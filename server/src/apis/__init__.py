from flask_restplus import Api

# aggregate the different namespaces / files here
from .raw_image_handler import api as rawImage
from .crop_image_handler import api as cropImage

api = Api(
    # Api metadata
    title='BYU AUVSI Imaging',
    version='0.1',
    description='REST API for BYU AUVSI Imaging',
)

api.add_namespace(rawImage)
api.add_namespace(cropImage)