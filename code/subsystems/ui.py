'''
UI subsystem
'''

import numpy
from matplotlib import pyplot as plt
from utils import constants as cst

class UISystem(object):
    '''
    Contains UI subsystem
    '''
    def __init__(self):
        self.d = 1

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
        count = 0
        with open(filepath+'.txt', 'r') as q:
            while True:
                line = q.readline()
                if not line:
                   break
                if count == 0:
                    [x0, y0, x1, y1] = line.split()
                    count += 1
                line_array = line.split()[1:]
                line_num = [float(i) for i in line_array]
                all_vals += line_num

        all_values = numpy.array(all_vals)
        num_points = len(all_values) / pt_dim
        points = all_values.reshape(num_points, pt_dim)

        Ldata = LineInputData()
        Ldata.lines = points
        Ldata.vertical_bounds = [y0, y1]
        Ldata.horizontal_bound = [x0, x1]
        return Ldata

    def drawDistribution(self, data_r0, data_r1):
        '''
        Draw the robot's paths, with solid as drawing and dotted
        as transport. Red = Robot 0, Blue = Robot 1
        @param data_r0 Path for Robot 0
        @param data_d1 Path for Robot 1
        '''
        path_r0 = numpy.array(data_r0)
        path_r1 = numpy.array(data_r1)
        fig = plt.figure()
        ax = plt.axes(xlim=(cst.LEFT_BORDER, cst.RIGHT_BORDER), 
                      ylim=(cst.BOTTOM_BORDER, cst.TOP_BORDER))
        for i in xrange(len(path_r0)-1):
            if i % 2 == 0:
                ax.plot([path_r0[i, 0], path_r0[i+1, 0]], 
                    [path_r0[i, 1], path_r0[i+1, 1]], lw=3, ls='--', color='red')
            else:
               ax.plot([path_r0[i, 0], path_r0[i+1, 0]],
                    [path_r0[i, 1], path_r0[i+1, 1]], lw=5, color='red')

        for i in xrange(len(path_r1)-1):
            if i % 2 == 0:
                ax.plot([path_r1[i, 0], path_r1[i+1, 0]],
                    [path_r1[i, 1], path_r1[i+1, 1]], lw=3, ls='--', color='blue')
            else:
               ax.plot([path_r1[i, 0], path_r1[i+1, 0]],
                    [path_r1[i, 1], path_r1[i+1, 1]], lw=5, color='blue')

        plt.show()
