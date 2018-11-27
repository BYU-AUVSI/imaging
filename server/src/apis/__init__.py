from flask_restplus import Api

# aggregate the different namespaces / files here
from .raw_image_handler import api as rawImage
from .crop_image_handler import api as cropImage
from .gps_handler import api as gps
from .state_handler import api as state
from .image_classification_handler import api as classification

api = Api(
    # Api metadata
    title='BYU AUVSI Imaging',
    version='0.1',
    description='REST API for BYU AUVSI Imaging: https://github.com/BYU-AUVSI/imaging',
)

api.add_namespace(gps)
api.add_namespace(state)
api.add_namespace(rawImage)
api.add_namespace(cropImage)
api.add_namespace(classification)