'''
Planner class
'''
from ui import UISystem
from scipy.spatial import distance
import numpy

class PlannerSystem(object):
    '''
    Contains planner subsystem
    '''
    def __init__(self):
        self.sys_ui = UISystem()
        self.start_r0 = [0, 0]
        self.start_r1 = [10, 10]

    def planTrajectories(self, filepath):
        '''
        Do lots of stuff..
        '''
        pathData = self.sys_ui.parseInputPaths(filepath)
        Distributor = DistributeWork(pathData, self.start_r0, self.start_r1)
        [path_r0, path_r1] = Distributor.getAllocation()
        self.sys_ui.drawDistribution(path_r0, path_r1)

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
        for i in xrange(len(self.lines)):
            print self.lines
            if cost_r0 > cost_r1:
                line_idx = self.findClosestLine(self.r1_set[-1])
                print 'R1', line_idx
                closest_line = self.lines[line_idx]
                self.lines = numpy.delete(self.lines, (line_idx), axis=0)
                (self.r1_set, c) = self.incoporateNewSegment(self.r1_set, closest_line)
                cost_r1 += c
            else:
                line_idx = self.findClosestLine(self.r0_set[-1])
                print 'R0', line_idx
                closest_line = self.lines[line_idx]
                self.lines = numpy.delete(self.lines, (line_idx), axis=0)
                (self.r0_set, c) = self.incoporateNewSegment(self.r0_set, closest_line)
                cost_r0 += c

        # Return the end
        self.r0_set += [self.r0_set[0]]
        self.r1_set += [self.r1_set[0]]
 
        return (self.r0_set, self.r1_set)

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

    def incoporateNewSegment(self, line_set, new_line):
        '''
        Take the line set, add the new line into the mix, include the
        transport from old line to new line.
        @param line_set The main set that we are incoporating into
        @param new_line The new line being added in
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
        return (line_set, (dist_start + dist_end))

class PathGeneration(object):
    '''
    Contain functions go from line coordinates
    to a path. I think we want this
    '''
    def __init__(self):
        pass
