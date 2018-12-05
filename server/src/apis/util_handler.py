from config import defaultConfigPath
from dao.util_dao import UtilDAO
from flask_restplus import Namespace, Resource


api = Namespace('util', description="Some utility methods for debugging")

@api.route('/reset/manual')
class UtilResetHandler(Resource):
    @api.doc(description="""
        WARNING! README!! This resets ALL images. This will remove all 
        cropped and classified images and reset any manual tapped images in
        incoming to False. Essentially this leaves you with a clean database
        as if a rosbag was just read in.""")
    @api.doc(responses={200:'OK', 500:'Reset failed'})
    def get(self):
        dao = UtilDAO(defaultConfigPath())
        dao.resetDB()
        
        return {'message': 'Success!'}