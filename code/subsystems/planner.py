'''
Planner class
'''
from ui import UISystem

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
        for i in xrange(len(self.all_lines)):
            print self.all_lines[i]
            if cost_r0 > cost_r1:
                new_set = self.r1_set
            else:
                new_set = self.r0_set
            cost = min(cost_r0, cost_r1)

            line_idx = self.findClosestLine(new_set[-1])
            closest_line = self.all_lines[line_idx]
            self.lines = numpy.array(self.lines, (line_idx), axis=0)
            cost += self.incoporateNewSegment(new_set, closest_line)
                
        return (self.r0_set, self.r1_set)

    def findClosestLine(self, current_point):
        '''
        Given the current point, find the nearest line (using either
        endpoint) of those remaining and return the index of that line
        @param current_point
        '''
        return NotImplementedError

    def incoporateNewSegment(self, line_set, new_line):
        '''
        Take the line set, add the new line into the mix, include the
        transport from old line to new line.
        @param line_set The main set that we are incoporating into
        @param new_line The new line being added in
        @param cost Add cost from the additional segments
        '''
        return NotImplementedError

class PathGeneration(object):
    '''
    Contain functions go from line coordinates
    to a path. I think we want this
    '''
    def __init__(self):
        pass
