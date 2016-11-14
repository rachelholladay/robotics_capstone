'''
Sketch of prototype planner
'''
import numpy, random

def readPoints(dir_path, filename):
    '''
    Pull all lines from log file. Currently assumes each line has four
    numbers: [(x1, y1), (x2, y2)].
    @param dir_path Directory containing file
    @param filename Name of file
    @return points List of all points
    '''
    all_vals = []
    pt_dim = 2*2
    with open(dir_path+filename+'.txt', 'r') as q:
        while True:
            line = q.readline()
            if not line:
                break
            line_array = line.split()
            line_num = [float(i) for i in line_array]
            all_vals += line_num

    all_values = numpy.array(all_vals)
    num_points = len(all_values) / pt_dim
    points = all_values.reshape(num_points, pt_dim)
    return points

def assign_random(points):
    '''
    Take the set of points and randomly assign them to each robot.
    @param points List of all points to be drawn
    @return zero_pts List of points for robot 0
    @return one_pts List of points for robot 1
    '''
    rob0 = []
    rob1 = []
    for i in xrange(len(points)):
        # Compute Random probability
        randSet = random.random()
        if randSet > 0.5:
            rob0.append(points[i])
        else:
            rob1.append(points[i])
    zero_pts = numpy.array(rob0)
    one_pts = numpy.array(rob1)
    return (zero_pts, one_pts)

def assign_partitionHalf(points, boundx):
    '''
    Take the set of points and split  them down the middle spatially
    with respect to the x direction.
    @param points List of all points to be drawn
    @param boundx Total x length of drawing area
    @return zero_pts List of points for robot 0
    @return one_pts List of points for robot 1
    '''
    rob0 = [] # Left size
    rob1 = [] # Right side
    halfx = boundx / 2.0
    for i in xrange(len(points)):
        x0 = points[i][0]
        x1 = points[i][2]
        if ((x0 < halfx) and (x1 < halfx)):
            rob0.append(points[i])
        elif ((x0 >= halfx) and (x1 >= halfx)):
            rob1.append(points[i])
        elif (x0 < halfx):
            rob0.append(points[i])
        elif (x0 >= halfx):
            rob1.append(points[i])
    zero_pts = numpy.array(rob0)
    one_pts = numpy.array(rob1)
    return (zero_pts, one_pts)        

def createPlan(lineSegs, start):
    '''
    @param lineSegs list of lines for the robot
    @param start The (x, y) starting position of the robot
    @return planSegs Contigous set of lines from start to each of the robot's 
                     lines (including transit between) and back to the start.
    @return planSegs Matching to planSegs that is boolean indicator on 
                     whether to draw or not
    '''
    numDrawSegs = len(lineSegs)
    numSegs = (2*numDrawSegs) + 1
    drawFlag = numpy.ones((numSegs)) * -1
    planSegs = numpy.zeros((numSegs, 4))

    prev = start
    for i in xrange(numDrawSegs):
        # Plan from prev to actual (no point)
        planSegs[(2*i)] = [prev[0], prev[1], lineSegs[i, 0], lineSegs[i, 1]]
        drawFlag[(2*i)] = False
        planSegs[(2*i)+1] = lineSegs[i, :]
        drawFlag[(2*i)+1] = True
        prev = [lineSegs[i, 2], lineSegs[i, 3]]
    planSegs[(numSegs-1)] = [prev[0], prev[1], start[0], start[1]]
    drawFlag[(numSegs-1)] = False
    return (planSegs, drawFlag)

def computeTiming(pathSegs, flags):
    '''
    @param pathSegs path of the robot
    @param flags corresponding flags of when the robot is drawing along path
    @return totalTime estimated time of path
    @return timeDrawing estimated time spent drawing
    '''
    totalTime = 0
    timeDrawing = 0
    timeFactor = 1
    for i in xrange(len(pathSegs)):
        p0 = pathSegs[i][0:1]
        p1 = pathSegs[i][2:3]
        dist = numpy.linalg.norm(p0-p1)
        timeDuration = dist * timeFactor
        totalTime += timeDuration
        if flags[i]:
            timeDrawing += timeDuration
    return (totalTime, timeDrawing)

def computeCollisions(path0, path1):
    return 0

if __name__ == "__main__":
    boundx = 10
    boundy = 10
    points = readPoints('drawingInputs/', 'test1')
    (rob0_pts, rob1_pts) = assign_partitionHalf(points, boundx)
    (p0, f0) = createPlan(rob0_pts, [0, 0])
    (p1, f1) = createPlan(rob1_pts, [boundx, boundy])
    (total0, draw0) = computeTiming(p0, f0)
    (total1, draw1) = computeTiming(p1, f1)
    print 'R0: {}. (Draw {})'.format(total0, draw0)
    print 'R1: {}. (Draw {})'.format(total1, draw1)
