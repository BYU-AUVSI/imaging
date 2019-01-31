from dao.model.outgoing_classification import outgoing_classification

class outgoing_autonomous(outgoing_classification):
    """
    Model class for the autonomous classification 'outgoing_autonomous' table. This model
    is only meant to hold datamembers that are distinct from the outgoing_manual
    table. Otherwise, all data members are found in its parent class: outgoing_classification
    """

    def __init__(self, tableValues=None, json=None):
        """
        Accepts various formats to instantiate this model object

        @type tableValues: [object]
        @param tableValues: List of table values, in table column order

        @type json: {object}
        @param json: Json dictionary of table values. Used by the REST API when receiving data
        """
        super(outgoing_autonomous, self).__init__(tableValues=tableValues, json=json)