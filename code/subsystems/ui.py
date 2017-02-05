'''
UI subsystem
'''

import numpy

class UISystem(object):
    '''
    Contains UI subsystem
    '''
    def __init__(self):
        pass

    def parseInputPaths(self, filepath):
        '''
        Given a filepath, read the file and convert into a planner-readable
        input. Currently assumes each line has four numbers:
        [(x1, y1), (x2, y2)].
        @param filepath Name of file
        @return points List of all points
        '''
        all_vals = []
        pt_dim = 2*2
        with open(filepath+'.txt', 'r') as q:
            while True:
                line = q.readline()
                if not line:
                   break
                line_array = line.split()[1:]
                line_num = [float(i) for i in line_array]
                all_vals += line_num

        all_values = numpy.array(all_vals)
        num_points = len(all_values) / pt_dim
        points = all_values.reshape(num_points, pt_dim)
        return points

