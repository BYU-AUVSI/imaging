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

@api.route('/reset')
class UtilResetHandler(Resource):
    @api.doc(description="""
        WARNING! README!! This resets the database. EVERYTHING. This will reset 
        the entire database to its clean, unused form. Note: this 
        will NOT delete the actual images themselves, only their references in the
        database. Removing an old image folder is left up to the user.""")
    @api.doc(responses={200:'OK', 500:'Reset failed'})
    def post(self):
        dao = UtilDAO(defaultConfigPath())
        dao.resetAll()

        return {'message': 'Success!'}

@api.route('/save')
class UtilResetSaveHandler(Resource):
    @api.doc(description="""
        This endpoint will save the current database into
        a set of csv files within the current server image directory (the directory
        where all the images currently held by the server are saved). This gives
        you the option of recovering the database at some later point (assuming
        you're able to get the image paths in the database to work properly).""")
    @api.doc(responses={200:'OK', 500:'Reset failed'})
    def post(self):
        dao = UtilDAO(defaultConfigPath())
        dao.saveAll()

        return {'message': 'Success!'}