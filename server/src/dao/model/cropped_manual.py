from dao.model.cropped_image import cropped_image

class cropped_manual(cropped_image):

    def __init__(self, tableValues=None, json=None):
        """
        Accepts various formats to instantiate this model object

        @type tableValues: [object]
        @param tableValues: List of table values, in table column order

        @type json: {object}
        @param json: Json dictionary of table values. Used by the REST API when receiving data
        """
        super(cropped_manual, self).__init__(tableValues=tableValues, json=json)