import re # regex to parse the point string

class point:
    """
    Represents a point datatype from postgres
    """

    # regex for extracting integer numbers from std point string
    # input string looks like: (42,22)
    INT_REGEX = r"[^\d]*(\d+)[^\d]*"

    def __init__(self, ptStr = None, x=None, y=None):
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
        return self._x

    @x.setter
    def x(self, x):
        self._x = x

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, y):
        self._y = y

    def toSql(self):
        if hasattr(self, '_x') and hasattr(self, '_y'):
            return "({},{})".format(self.x, self.y)
        return None

    def toDict(self):
        if hasattr(self, '_x') and hasattr(self, '_y'):
            dict = {}
            dict['x'] = self.x
            dict['y'] = self.y
            return dict
        return None

    def __str__(self):
        str = self.toSql()
        return '' if str is None else str 
