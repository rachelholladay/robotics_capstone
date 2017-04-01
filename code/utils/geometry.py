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

    def __str__(self):
        return "Point: x:%f y:%f" % (self.x, self.y)

    def __sub__(self, pt):
        """
        Subtracts Point from current object.
        Returns Point containing the difference in x,y
        @param pt Point object to subtract
        @return Point(self.x - pt.x, self.y - pt.y)
        """
        return Point(self.x - pt.x, self.y - pt.y)

class DirectedPoint(Point):
    """
    A DirectedPoint is a point with orientation. The expected use is for
    localization data, in which objects are represented by an (x,y,theta)
    """
    def __init__(self, x=None, y=None, theta=None, valid=True):
        Point.__init__(self, x, y, valid)
        self.theta = theta

    def __str__(self):
        return "DirectedPoint x:%f y:%f theta:%f" % \
            (self.x, self.y, self.theta)

    def __sub__(self, dpt):
        """
        Subtracts the DirectedPoint from the current object.
        Returns a DirectedPoint containing the difference:
        @param dpt Directed point to subtract
        @return DirectedPoint(self.x-dpt.x, self.y-dpt.y, self.theta-dpt.theta)
        """
        return DirectedPoint(
            self.x - dpt.x,
            self.y - dpt.y,
            self.theta - dpt.theta)
