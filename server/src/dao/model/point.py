import re # regex to parse the point string

class point:
    """
    Represents a point datatype from postgres. Used by manual_cropped model for 
    crop_coordinates
    """

    # regex for extracting integer numbers from std point string
    # input string looks like: (42,22)
    INT_REGEX = r"[^\d]*(\d+)[^\d]*"

    def __init__(self, ptStr = None, x=None, y=None):
        """
        Provides various ways to initialize different point types

        @type ptStr: string
        @param ptStr: String of a integer point, should look something like: "(45,56)"

        @type x: int
        @param x: Integer for the x component of the point

        @type y: int
        @param y: Integer for the y component of the point
        """
        if x is not None and y is not None:
            self.x = x
            self.y = y
        elif ptStr is not None:
            unformattedNums = ptStr.split(',')
            if len(unformattedNums) > 1:
                intMatcher = re.compile(self.INT_REGEX)
                self.x = int(intMatcher.match(unformattedNums[0]).group(1))
                self.y = int(intMatcher.match(unformattedNums[1]).group(1))

    @property
    def x(self):
        """
        X component of the point
        """
        return self._x

    @x.setter
    def x(self, x):
        self._x = x

    @property
    def y(self):
        """
        Y component of the point
        """
        return self._y

    @y.setter
    def y(self, y):
        self._y = y

    def toSql(self):
        """
        Generate a string that can be successfully inserted as a point into postgres.
        Requires both x and y attributes to be present. 

        @rtype: string
        @return: String representing the point. Formatted: (x,y). If x or y is not present, None.
        """
        if hasattr(self, '_x') and hasattr(self, '_y'):
            return "({},{})".format(self.x, self.y)
        return None

    def toDict(self):
        """
        Return attributes contained in this model as a dictionary

        @rtype: {int}
        @return: String dictionary of point properties. If x or y is not present, None
        """
        if hasattr(self, '_x') and hasattr(self, '_y'):
            dict = {}
            dict['x'] = self.x
            dict['y'] = self.y
            return dict
        return None

    def __str__(self):
        """
        Debug convenience method to get this instance as a string
        """
        str = self.toSql()
        return '' if str is None else str 
