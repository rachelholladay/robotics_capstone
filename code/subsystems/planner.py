'''
Planner class
'''
from scipy.spatial import distance
from utils import constants, geometry
import numpy

class PlannerSystem(object):
    '''
    Contains planner subsystem
    '''
    def __init__(self):
        self.start_r0 = [constants.BOTTOM_BORDER, constants.LEFT_BORDER]
        self.start_r1 = [constants.TOP_BORDER, constants.RIGHT_BORDER]

    def planTrajectories(self, data):
        '''
        Do lots of stuff..
        '''
        pathData = self.scaleData(data)
        Distributor = DistributeWork(pathData, self.start_r0, self.start_r1)
        (bluePath, badPath) = Distributor.getAllocation()
        return (bluePath, badPath)

    def scaleData(self, data):
        '''
        Scale data to the bounds.
        '''
        #TODO add padding
        [start_y, end_y] = data.vertical_bounds
        [start_x, end_x] = data.horizontal_bounds
        inputs = data.lines
        pathData = numpy.zeros((inputs.shape))
        for i in xrange(4):
           pts = inputs[:, i]
           if i % 2 == 0:
              init = float(end_x - start_x)
              final = float(constants.RIGHT_BORDER - constants.LEFT_BORDER)
              pts_d = numpy.divide(pts, init)
              pts_m = numpy.multiply(pts_d, final)
              pathData[:, i] = numpy.add(pts_m, constants.LEFT_BORDER)
           else:
              init = float(end_y - start_y)
              final = float(constants.TOP_BORDER - constants.BOTTOM_BORDER)
              pts_d = numpy.divide(pts, init)
              pts_m = numpy.multiply(pts_d, final)
              pathData[:, i] = numpy.add(pts_m, constants.BOTTOM_BORDER) 
        return pathData

class DistributeWork(object):
    '''
    Given drawing data, split it betweeo two robots
    '''
    def __init__(self, pathData, start_r0, start_r1):
        self.lines = pathData
        self.r0_set = [start_r0]
        self.r1_set = [start_r1]

    def getAllocation(self):
        '''
        Iterate through all the lines, assigning them to a robot, reorder for
        efficiency and return the resulting allocation
        '''
        cost_r0 = 0
        cost_r1 = 0
        writing_r0 = []
        writing_r1 = []
        for i in xrange(len(self.lines)): 
            if cost_r0 > cost_r1:
                line_idx = self.findClosestLine(self.r1_set[-1])
                closest_line = self.lines[line_idx]
                self.lines = numpy.delete(self.lines, (line_idx), axis=0)
                (self.r1_set, drawingFlags, c) = self.addNewSegment(self.r1_set, closest_line)
                writing_r1 += [drawingFlags]
                cost_r1 += c
            else:
                line_idx = self.findClosestLine(self.r0_set[-1])
                closest_line = self.lines[line_idx]
                self.lines = numpy.delete(self.lines, (line_idx), axis=0)
                (self.r0_set, drawingFlags, c) = self.addNewSegment(self.r0_set, closest_line)
                writing_r0 += [drawingFlags]
                cost_r0 += c

        # Return the end
        self.r0_set += [self.r0_set[0]]
        self.r1_set += [self.r1_set[0]]
        writing_r0 += [False]
        writing_r1 += [False]

        bluePath = geometry.DirectedPath(path_r0, writing_r0)
        badPath = geometry.DirectedPath(path_r1, writing_r1)

        return (bluePath, badPath)

    def findClosestLine(self, current_point):
        '''
        Given the current point, find the nearest line (using either
        endpoint) of those remaining and return the index of that line
        @param current_point
        '''
        (num_points, _) = self.lines.shape
        twod_points = self.lines.reshape((num_points*2, 2))
        dists = distance.cdist([current_point], twod_points, 'euclidean')[0]
        idx = numpy.argmin(dists)
        if idx % 2 == 1:
            idx -= 1
        return (idx  / 2)

    def addNewSegment(self, line_set, new_line):
        '''
        Take the line set, add the new line into the mix, include the
        transport from old line to new line.
        @param line_set The main set that we are incoporating into
        @param new_line The new line being added in
        @param drawingFlags marks when to draw or not draw
        @param cost Add cost from the additional segments
        '''
        last_point = line_set[-1]
        start = new_line[0:2]
        dist_start = numpy.linalg.norm(last_point - start)
        end = new_line[2:]
        dist_end = numpy.linalg.norm(last_point - end)
        if dist_start <= dist_end:
            line_set += [start.tolist()]
            line_set += [end.tolist()]
        else:
            line_set += [end.tolist()]
            line_set += [start.tolist()]
        #FIXME adjust to be accurate
        drawingFlags = [False, True]
        return (line_set, drawingFlags, (dist_start + dist_end))

class PathGeneration(object):
    '''
    Contain functions go from line coordinates
    to a path. I think we want this
    '''
    def __init__(self):
        pass
