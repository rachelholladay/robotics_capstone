"""
UI subsystem
Enables the UI to show progress and parse input from the files
"""

import numpy

from utils import constants as cst
from utils import dataStorage as ds

class UISystem(object):
    """
    UI subsystem operation class
    """
    def __init__(self):
        """
        Sets up initialization parameters
        """
        self.d = 1

    def parseInputPaths(self, filepath):
        """
        Given a filepath, read the file and convert into a planner-readable
        input. Currently assumes each line has four numbers:
        [(x1, y1), (x2, y2)].

        @param filepath Name of file
        @return points List of all points
        """
        all_vals = []
        pt_dim = 2 * 2
        count = 0
        with open(filepath + '.txt', 'r') as q:
            while True:
                line = q.readline()
                if not line:
                   break
                if count == 0:
                    [x0, y0, x1, y1] = line.split()
                    count += 1
                else:
                    line_array = line.split()[1:]
                    line_num = [float(i) for i in line_array]
                    all_vals += line_num

        all_values = numpy.array(all_vals)
        num_points = len(all_values) / pt_dim
        points = all_values.reshape(num_points, pt_dim)

        Ldata = ds.LineInputData()
        Ldata.lines = points
        Ldata.vertical_bounds = [int(y0), int(y1)]
        Ldata.horizontal_bounds = [int(x0), int(x1)]
        
        return Ldata

    def drawDistribution(self, bluePath, badPath):
        """
        Draw the robot's paths, with solid as drawing and dotted
        as transport. 

        @param bluePath Path for Blue the Robot
        @param badPath Path for Bad the Robot
        """
        from matplotlib import pyplot as plt

        fig = plt.figure()
        ax = plt.axes(xlim=(cst.LEFT_BORDER, cst.RIGHT_BORDER), 
                      ylim=(cst.BOTTOM_BORDER, cst.TOP_BORDER))
        ls = ['--', '-']
        lw = [3, 5]
        for i in xrange(bluePath.length - 1):
            p0 = (bluePath[i]).target
            p1 = (bluePath[i + 1]).target
            ax.plot([p0.x, p1.x], [p0.y, p1.y], lw=lw[(i % 2)],
                    ls=ls[(i % 2)], color='blue')

        for i in xrange(badPath.length-1):
            p0 = (badPath[i]).target
            p1 = (badPath[i+1]).target
            ax.plot([p0.x, p1.x], [p0.y, p1.y], lw=lw[(i % 2)],
                    ls=ls[(i % 2)], color='red')

        plt.show()
