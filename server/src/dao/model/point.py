class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getX(self):
        return self.x

    def setX(self, x):
        self.x = x

    def getY(self):
        return self.y

    def setY(self, y):
        self.y = y

    def toSql(self):
        return "'({},{})'".format(self.x, self.y)

    x = property(getX, setX)
    y = property(getY, setY)