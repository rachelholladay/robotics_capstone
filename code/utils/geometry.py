'''
Classes for geometric figures and representations
'''
import math
import constants as cst

class Point(object):
    """
    Class representing a point in 2D space
    """
    def __init__(self, x=None, y=None, valid=True):
        self.x = x
        self.y = y
        self.valid = valid

    def __str__(self):
        return "Point: x:{:.3f} y:{:.3f}".format(self.x, self.y)

    def __sub__(self, pt):
        """
        Subtracts Point from current object.
        Returns Point containing the difference in x,y
        @param pt Point object to subtract
        @return Point(self.x - pt.x, self.y - pt.y)
        """
        return Point(self.x - pt.x, self.y - pt.y)

    def __add__(self, pt):
        """
        Adds pt to current object self + pt. Element-wise operation.
        @param pt Point object to add
        @return Point object
        """
        return Point(self.x + pt.x, self.y + pt.y)

class DirectedPoint(Point):
    """
    A DirectedPoint is a point with orientation. The expected use is for
    localization data, in which objects are represented by an (x,y,theta)
    """
    def __init__(self, x=None, y=None, theta=None, valid=True):
        Point.__init__(self, x, y, valid)
        self.theta = theta

        __rmul__ = self.__mul__  # define multiplication bidirectionaly

    def __str__(self):
        return "DPT ({:.3f}, {:.3f}, {:.3f})".format(self.x, self.y, self.theta)

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

    def __add__(self, dpt):
        """
        Element-wise addition of DirectedPoint objects.
        @param dpt DirectedPoint to add
        @return DirectedPoint object
        """
        return DirectedPoint(
            self.x + dpt.x, 
            self.y + dpt.y,
            self.theta + dpt.theta)

    def dot(self, dpt):
        """
        Dot product of (x,y) components
        @param dpt Directed Point to use in dot product
        @return self.x * dpt.x + self.y + dpt.y
        """
        return (self.x * dpt.x) + (self.y * dpt.y)

    def __mul__(self, cst):
        """
        Multiplies a constant by the (x,y) components.
        @param cst Constant to multiply.
        @return DirectedPoint with (x,y) scaled by cst
        """
        return DirectedPoint(
            self.x * cst,
            self.y * cst,
            self.theta)

    def __rmul__(self, cst):
        """
        Multiplies a constant by the (x,y) components.
        @param cst Constant to multiply.
        @return DirectedPoint with (x,y) scaled by cst
        """
        return DirectedPoint(
            self.x * cst,
            self.y * cst,
            self.theta)

    def dist(self, dpt):
        """
        Gets L2 norm between (x,y) of self and given directedpoint
        @param dpt DirectedPoint to find L2 distance between
        """
        return math.sqrt(
            math.pow(self.x - dpt.x, 2) + math.pow(self.y - dpt.y, 2))


class DirectedPath(object):
    """
    A DirectedPath is a list of DirectedPoints that make up the path
    """
    def __init__(self, path_array, writing_array):
        self.path = []
        for i in xrange(len(path_array)):
            d = DirectedPoint(x=path_array[i][0], y=path_array[i][1], theta=0)
            w = Waypoint(d, writing_array[i])
            self.path.append(w)

        self.length = len(self.path)

    def __str__(self):
        message = ''
        for i in xrange(len(self.path)):
            message += '{} : {}\n'.format(i, self.path[i])
        return message

    def __getitem__(self, key):
        """
        Returns DirectedPoint path value at index key
        @param key Index to get path at
        """
        return self.path[key]

class Waypoint(object):
    """
    Contains waypoint data, including the target XY point and the writing status
    for the duration of motion towards that waypoint
    """
    def __init__(self, target, write_status):
        """
        @param target DirectedPoint containing XY coordinate for waypoint
        @param status of writing implement - uses cst.WRITE_ENABLE or 
                cst.WRITE_DISABLE
        """
        self.target = target
        self.write_status = write_status

    def __str__(self):
        message = str(self.target)
        message += ", "
        if self.write_status is cst.WRITE_ENABLE:
            message += "WRITE_ENABLED"
        elif self.write_status is cst.WRITE_DISABLE:
            message += "WRITE_DISABLED"
        return message
