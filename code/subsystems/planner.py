'''
Planner class
'''
class PlannerSystem(object):
    '''
    Contains planner subsystem
    '''
    def __init__(self):
        pass

    def planTrajectories(self, filepath):
        '''
        Do lots of stuff..
        '''

        pathData = ui.parseInputPaths(filepath)
        Distributor = DistributeWork(pathData)
        [path_r0, path_r1] = self.getAllocation()

class DistributeWork(object):
    '''
    Given drawing data, split it betweeo two robots
    '''
    def __init__(self, pathData):
        self.all_lines = pathData
        self.r0_set = []
        self.r1_set = []

    def getAllocation(self):
        '''
        Iterate through all the lines, assigning them to a robot, reorder for
        efficiency and return the resulting allocation
        '''
        for i in xrange(len(self.all_lines)):
           # Greedy algo, iterate through..
        self.reorderSet(self.r0_set)
        self.reorderSet(self.r1_set)
        return (self.r0_set, self.r1_set)

    def assignLine(self, line):
        '''
        '''
        return NotImplementedError

class PathGeneration(object):
    '''
    Contain functions go from line coordinates
    to a path. I think we want this
    '''
    def __init__(self):
        pass
