'''
Sketch of prototype planner
'''
import numpy, random, math
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation

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

def ptsMatching(p0, p1, flagBool):
    '''
    @param p0 Starting point pair
    @param p1 End point pair
    @param flagBool Flag on whether drawing
    @return segPts interpolated path points
    @return segFlag interpolate path flags
    '''
    sub = numpy.subtract(p0, p1)
    dist = int(math.ceil(numpy.linalg.norm(sub)))
    xvals = numpy.linspace(p0[0], p1[0], dist, endpoint=flagBool)
    yvals = numpy.linspace(p0[1], p1[1], dist, endpoint=flagBool)
    if not flagBool:
        xvals = xvals[1:]
        yvals = yvals[1:]
    segPts = zip(xvals, yvals)
    segFlag = [flagBool]*len(segPts)
    return (segPts, segFlag)

def createPlan(segs, start):
    '''
    @param segs list of lines for the robot
    @param start The (x, y) starting position of the robot
    @return planSegs Contigous set of lines from start to each of the robot's 
                     lines (including transit between) and back to the start.
    @return flagsList Matching to planSegs that is boolean indicator on 
                     whether to draw or not
    '''
    numDrawSegs = len(segs)
    pts = []
    flags = []
    prev = start
    for i in xrange(numDrawSegs):
        # Plan from prev to actual (no point)
        segPtsF, segFlagF = ptsMatching(prev, segs[i, 0:2], True)
        pts.append(segPtsF)
        flags.append(segFlagF)

        # Plan through path
        segPtsT, segFlagT = ptsMatching(segs[i, 0:2], segs[i, 2:], True)
        pts.append(segPtsT)
        flags.append(segFlagT)
        prev = segs[i, 2:]

    segPtsL, segFlagL = ptsMatching(prev, start, False)
    pts.append(segPtsL)
    flags.append(segFlagL)
    pts.append([start])
    flags.append([False])
    flagsList = numpy.array([item for sublist in flags for item in sublist]) 
    ptsList = numpy.array([item for sublist in pts for item in sublist])
    return (ptsList, flagsList)

def computeTiming(pathSegs, flags):
    '''
    @param pathSegs path of the robot
    @param flags corresponding flags of when the robot is drawing along path
    @return timeList time stamps along path
    @return timeDrawing estimated time spent drawing
    '''
    #TODO might need to double check timing
    timeList = numpy.zeros((len(pathSegs)))
    totalTime = 0
    timeDrawing = 0
    timeFactor = 1
    for i in xrange(1, len(pathSegs)):
        sub = numpy.subtract(pathSegs[(i-1), :], pathSegs[i, :])
        dist = numpy.linalg.norm(sub)
        totalTime += (timeFactor*dist)
        timeList[i] = totalTime
        if flags[i]:
            timeDrawing += (timeFactor*dist)
    return (timeList, timeDrawing)

def computeCollisions(path0, time0, path1, time1, distCheck):
    '''
    @param path0 robot 0's path
    @param time0 robot 0's time mapping
    @param path1 robot 1's path
    @param time1 robot 1's time mapping
    @param distCheck Minimum allowable distance to be collision free
    @returns Boolean Flag, true if in collision
    '''
    if time0[-1] > time1[-1]:
        timeMaster = time1
        pathMaster = path1
        timeSlave = time0
        pathSlave = path0
    else:
        timeMaster = time0
        pathMaster = path0
        timeSlave = time1
        pathSlave = path1
     
    for t in xrange(len(timeMaster)):
        t0 = timeMaster[t]
        t1 = (numpy.abs(timeSlave-t0)).argmin()
        sub = numpy.subtract(pathMaster[t], pathSlave[t1])
        dist = numpy.linalg.norm(sub)
        if dist < distCheck:
            return True
    return False

def animatePlanner(plan0, flag0, time0):
    return 0

def init():
    for line in lines:
        line.set_data([],[])
    return lines

def animate(i, path0, path1):
    x0 = path0[0:i, 0]
    y0 = path0[0:i, 1]
    x1 = path1[0:i, 0]
    y1 = path1[0:i, 1]
    xlist = [x0, x1]
    ylist = [y0, y1]
    for lnum,line in enumerate(lines):
        line.set_data(xlist[lnum], ylist[lnum]) # set data for each line separately. 
    return lines    

if __name__ == "__main__":
    boundx = 10
    boundy = 10
    safeDist = 1
    points = readPoints('drawingInputs/', 'test2')
    (rob0_pts, rob1_pts) = assign_partitionHalf(points, boundx)
    (p0, f0) = createPlan(rob0_pts, [0, 0])
    (p1, f1) = createPlan(rob1_pts, [boundx, boundy])
    (t0, timeDraw0) = computeTiming(p0, f0)
    (t1, timeDraw1) = computeTiming(p1, f1)
    print 'R0: {} / {}'.format(timeDraw0, t0[-1])
    print 'R1: {} / {}'.format(timeDraw1, t1[-1])
    collision = computeCollisions(p0, t0, p1, t1, safeDist)
    if collision: 
        print 'Paths Collide'
    else:
        print 'Collision Free'


    # Animation
    plt.rcParams['animation.ffmpeg_path'] ='/usr/bin/ffmpeg'
    FFwriter = animation.FFMpegWriter()
    fig = plt.figure()
    ax = plt.axes(xlim=(0, boundx), ylim=(0, boundy))
    for i in xrange(len(points)):
        ax.plot([points[i, 0], points[i, 2]], [points[i, 1], points[i, 3]], lw=5, color='black') 
    line, = ax.plot([], [], lw=2)

    plotcols = ["orange", "red"]
    lines = []
    for index in range(2):
        lobj = ax.plot([],[],lw=2, color=plotcols[index])[0]
        lines.append(lobj)

    count = max(len(p0), len(p1))+1
    anim = animation.FuncAnimation(fig, animate, init_func=init, fargs=[p0, p1],
                               frames=count, interval=500, blit=True)
    anim.save('plannerOutput/basic_animation.mp4', writer=FFwriter) 
    plt.show()
