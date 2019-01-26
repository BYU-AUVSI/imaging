from dao.model.cropped_image import cropped_image

class cropped_autonomous(cropped_image):
    """
    Model class for the cropped_autonomous table. All values common to both 
    the manual and autonomous cropped tables are defined in the parent 'cropped_image'
    class. This child class should only be modified if there are values unique to 
    the cropped_autonomous table  (that are not in the cropped_manual table)
    """

    def __init__(self, tableValues=None, json=None):
        """
        Accepts various formats to instantiate this model object

        @type tableValues: [object]
        @param tableValues: List of table values, in table column order

        @type json: {object}
        @param json: Json dictionary of table values. Used by the REST API when receiving data
        """
        pass