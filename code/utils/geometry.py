'''
Classes for geometric figures and representations
'''
class Point(object):
    """
    Class representing a point in 2D space
    """
    def __init__(self, x=None, y=None, valid=True):
        self.x = x
        self.y = y
        self.valid = valid

class DirectedPoint(Point):
    """
    A DirectedPoint is a point with orientation. The expected use is for
    localization data, in which objects are represented by an (x,y,theta)
    """
    def __init__(self, x=None, y=None, theta=None, valid=True):
        Point.__init__(self, x, y, valid)
        self.theta = theta


