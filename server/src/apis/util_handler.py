from config import defaultConfigPath
from dao.util_dao import UtilDAO
from flask_restplus import Namespace, Resource


api = Namespace('util', description="Some utility methods for development/debugging")

@api.route('/reset/manual')
class UtilManualResetHandler(Resource):
    @api.doc(description="""
        WARNING! README!! This resets ALL images. This will remove all 
        cropped and classified images and reset any manual tapped images in
        incoming to False. Essentially this leaves you with a clean database
        as if the rostopics were just read in (meaning images aren't entirely 
        erased and the incoming_image table is not truncated)""")
    @api.doc(responses={200:'OK', 500:'Reset failed'})
    def get(self):
        dao = UtilDAO(defaultConfigPath())
        dao.resetManualDB()
        
        return {'message': 'Success!'}

@api.route('/reset/autonomous')
class UtilAutoResetHandler(Resource):
    @api.doc(description="""
        WARNING! README!! This resets ALL images. This will remove all 
        autonomous classified images and reset any autonomous tapped images in
        incoming to False. Essentially this leaves you with a clean database
        as if the rostopics were just read in (meaning images aren't entirely 
        erased and the incoming_image table is not truncated)""")
    @api.doc(responses={200:'OK', 500:'Reset failed'})
    def get(self):
        dao = UtilDAO(defaultConfigPath())
        dao.resetAutonomousDB()

        return {'message': 'Success!'}

