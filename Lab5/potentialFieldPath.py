import numpy as np
from potentialFieldStep import potentialFieldStep
from copy import deepcopy
import sys
import random
from sys import path
from os import getcwd
from calculateFK import calculateFK
from matplotlib import pyplot as plt

def potentialFieldSegment(map, qStart, qGoal, params):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param qStart:       start pose of the robot (1x6).
    :param qGoal:        goal pose of the robot (1x6).
    :return:            returns an Nx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix.
    """
    thresh = 0.04
    rrt_index = -1

    ipath, path, isDoneArray = [], [], []
    qNext, delnan = potentialFieldStep(qStart,map,qGoal, params)

    # Joint 5 and 6 are not affected by potential fields
    qNext[4]=qGoal[4]
    qNext[5]=qGoal[5]

    path.append(qNext)
    ipath.append(qNext)
    
    f=0
    isDone=False
    rrt_count = 0
    
    while (isDone==False):
        f+=1
        qCurr=ipath[-1]
        qNext,delnan=potentialFieldStep(qCurr,map,qGoal, params)
        qNext[4] = qGoal[4]
        qNext[5] = qGoal[5]
        ipath.append(qNext)

        if(delnan==True):
            del(ipath[-1])
  
        toAppend = ((ipath[-1] + ipath[-2])/2)
        if(np.abs(np.sum(toAppend - path[-1])) > 0.005):
            path.append(toAppend)

        if(f%1000==0):
            print("iteration #:",f)
        
        g=0
        for r in range(0,3):
            if abs(qGoal[r]-path[-1][r])<thresh:
                g+=1
        if g==3:
            print("goal reached")
            path.append(qGoal)
            isDone = True

        if f>=100 and (isDone == False):
            if(rrt_count > 99):
                print("RRT was not able to reach the goal in numPoints")
                print(path[-1])
                return path
                return []
            
            if(rrt_index != -1):
                path = path[:rrt_index]

            rrt_index = len(path)

            print("RRT point appended at index " + str(len(path)))
            rrt_path = rrt(map, path[-1], 1)
            if(len(rrt_path) > 0): 
                rrt_path = rrt_path[0]
            else:
                return []
    
            path.append(rrt_path)
            ipath.append(rrt_path)
            f = 0
            rrt_count += 1

    return path


def potentialFieldPath(map, qStart, qGoal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param qStart:       start pose of the robot (1x6).
    :param qGoal:        goal pose of the robot (1x6).
    :return:            returns an Nx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix.
    """

    params[0] =10e5
    params[1] =10
    params[2] =100
    params[3] =0.1

    #eta = params[0] 
    #zeta = params[1] 
    #rho0 = params[2] 
    #alpha = params[3] 

    path1 = potentialFieldSegment(map, qStart, qGoal, params)
    print(path1)
    return path1


    # Linear search through parameter space
    eta_search = np.linspace(10e3, 10e8, 50)
    rho_search = np.linspace(10, 100,50)
    path_lens = []
    param_tried = []

    #for eta in eta_search:
    #    params[0] = eta
    #    param_tried.append(eta)
    #    path1 = potentialFieldSegment(map, qStart, qGoal, params)
    #    path_lens.append(len(path1))


    #for rho0 in rho_search:
    #    params[2] = rho0
    #    param_tried.append(rho0)
    #    path1 = potentialFieldSegment(map, qStart, qGoal, params)
    #    path_lens.append(len(path1))

    #print(zip(param_tried, path_lens))

    #plt.scatter(param_tried, path_lens)
    #plt.show()


def boundaryCollision(point, boundary):
    outOfBounds = False
    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(point[0])

    for i in range(len(startPos)):

        # Check if joint i is within the boundary
        outOfBounds |= (startPos[i][0] < boundary[0])
        outOfBounds |= (startPos[i][1] < boundary[1])
        outOfBounds |= (startPos[i][2] < boundary[2])
        outOfBounds |= (startPos[i][0] > boundary[3])
        outOfBounds |= (startPos[i][1] > boundary[4])
        outOfBounds |= (startPos[i][2] > boundary[5])

    return outOfBounds

def obstacleCollision(start, goal, obstacles):
    """
    start is a pose containing q1 -> qe
    goals is a pose containing q1 -> qe
    returns if there is a feasible straight line path
    """
    lineCollision = False
    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(start[0])
    goalPos, _ = f.forward(goal[0])

    for obstacle in obstacles:
        # Iterate through every joint
        for i in range(len(startPos)):
            results = detectCollision([startPos[i]], [goalPos[i]], obstacle)
            for result in results:
                lineCollision |= result

    return lineCollision

def postProcessing(points, obstacles):
    if len(points)<=5:
        return points
    processed = deepcopy(points)

    efficiency = 0
    i = 0
    maxIter = len(points)
    while(i < maxIter):

        a = random.randrange(1, len(processed) - 1)
        b = random.randrange(1, len(processed) - 1)

        # b is always larger than a
        if b == a: continue
        if(b < a): a, b = b, a

        if(a == b): continue
        coll = obstacleCollision([processed[a]], [processed[b]], obstacles)

        if not coll:
            efficiency += b - a -1

            y = 0
            for x in range(len(processed)) :
                if x <= a or x >= b :
                    processed[y] = processed[x]
                    y += 1
            processed = processed[:len(processed) - (b - a - 1)]
        i += 1

    # verify collision free path exists
    verificationFlag = True
    verificationFlag &= (points[0] == processed[0])
    verificationFlag &= (points[-1] == processed[-1])
    for i in range(len(processed) - 1):
        if obstacleCollision([processed[i]], [processed[i + 1]], obstacles):
            verificationFlag &= False

    if not verificationFlag:
        print ("Something is wrong in post-processing")
        print(processed)

    return processed

def rrt(map, start, numPoints):
    obstacles = deepcopy(map.obstacles)
    base1 = np.array([[-30,-30,0.5,-0.1,-0.1,80]])
    base2 = np.array([[0.1,0.1,0.1,30,30,80]])
    boundary = deepcopy(map.boundary)
    
    # tunable parameter related to rho0
    bufferRadius = 0

    for obstacle in obstacles:
        obstacle[0] = max(obstacle[0] - bufferRadius, boundary[0])
        obstacle[1] = max(obstacle[1] - bufferRadius, boundary[1])
        obstacle[2] = max(obstacle[2] - bufferRadius, boundary[2])
        obstacle[3] = min(obstacle[3] + bufferRadius, boundary[3])
        obstacle[4] = min(obstacle[4] + bufferRadius, boundary[4])
        obstacle[5] = min(obstacle[5] + bufferRadius, boundary[5])

    # check if start or goal pose is inside a C-space obstacle
    startObstacle = obstacleCollision([start], [start], obstacles)
    obstacles=np.append(obstacles,base1,axis=0)
    obstacles=np.append(obstacles,base2,axis=0)
    if(startObstacle):
        print("Target or Start inside obstacle")
        return ([])

    if(len(obstacles) != 0):
        obstacles=np.append(obstacles,base1,axis=0)
        obstacles=np.append(obstacles,base2,axis=0)
    else:
        obstacles=base1
        obstacles=np.append(obstacles,base2,axis=0)

    # in the list
    points = []
    maxIter = 10
    i = 0

    # Lower joint limits in radians (grip in mm
    # (negative closes more firmly))
    lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]

    # Upper joint limits in radians (grip in mm)
    upperLim = [1.4, 1.4, 1.7, 1.7, 1.5, 30]

    currentPose = start

    while (i < maxIter):
        # sample a pose
        randQ1 = random.uniform(lowerLim[0], upperLim[0])
        randQ2 = random.uniform(lowerLim[1], upperLim[1])
        randQ3 = random.uniform(lowerLim[2], upperLim[2])
        randQ4 = random.uniform(lowerLim[3], upperLim[3])
        randQE = random.uniform(lowerLim[4], upperLim[4])

        newPose = [randQ1, randQ2, randQ3, randQ4, randQE, 0]

        coll = obstacleCollision([currentPose],[newPose], obstacles)
        coll |= boundaryCollision([currentPose], boundary)

        if not coll:
            points.append(list(newPose))
            currentPose = newPose
            i += 1
        if i==numPoints:
            break

    return points

def detectCollision (linePt1, linePt2, box):
    """
    Check if multiple lines formed from two points intercepts with the block.
    Check one at a time.
    :param linePt1: [n,3] np array where each row describes one end of a line
    :param linePt2  [n,3] np array where each row describes one end of a line
    :param box [xmin, ymin, zmin, xmax, ymax, zmax]: box
    :return: n dimensional array, true if line n is in collision with the box
    """
    n_samples = len(linePt1)
    return [detectCollisionOnce(linePt1[index], linePt2[index], box) for index in range(n_samples)]

def detectCollisionOnce(linePt1, linePt2, box):
    """
    Check if line form from two points intercepts with the per block.
    Check one at a time.
    :param linePt1 [x,y,z]:
    :param linePt2 [x,y,z]:
    :param box [xmin, ymin, zmin, xmax, ymax, zmax]:
    :return: true if collision, otherwise false
    """
    # %% Initialization
    # box = box[0]
    # Initialize all lines as collided.
    isCollided = np.ones(1)

    # Divide box into lower left point and "size"
    boxPt1 = np.array([box[0],box[1], box[2]])
    # Create point in the opposize corner of the box
    boxPt2 = np.array([box[3],box[4], box[5]])
    boxSize = boxPt2 - boxPt1
    # Find slopes vector
    lineSlope = linePt2 - linePt1
    lineSlope = [0.001 if num == 0 else num for num in lineSlope]

    # Begin Collision Detection

    # The parameter t = [0,1] traces out from linePt1 to linePt2

    # Return false if box is invalid or has a 0 dimension
    if min(boxSize) <= 0:
        isCollided = 0 * isCollided
        return isCollided

    # Get minimum and maximum intersection with the y-z planes of the box
    txmin = (boxPt1[0] - linePt1[0]) / lineSlope[0]
    txmax = (boxPt2[0] - linePt1[0]) / lineSlope[0]

    # Put them in order based on the parameter t
    ts = np.sort(np.array([txmin,txmax]).transpose())
    txmin = ts[0]
    txmax = ts[1]


    # Get minimum and maximum intersection with the x-z planes of the box

    tymin = (boxPt1[1] - linePt1[1]) / lineSlope[1]
    tymax = (boxPt2[1] - linePt1[1]) / lineSlope[1]

    # Put them in order based on the parameter t
    ts = np.sort(np.array([tymin, tymax]).transpose())
    tymin = ts[0]
    tymax = ts[1]
    # if we miss the box in this plane, no collision
    isCollided = np.logical_and(isCollided, np.logical_not(np.logical_or((txmin > tymax), (tymin > txmax))))

    # identify the parameters to use with z
    tmin = np.maximum.reduce([txmin, tymin])
    tmax = np.minimum.reduce([txmax, tymax])

    # Get minimum and maximum intersection with the x-z planes of the box
    tzmin = (boxPt1[2] - linePt1[2]) / lineSlope[2]
    tzmax = (boxPt2[2] - linePt1[2]) / lineSlope[2]
    # Put them in order based on the parameter t
    ts = np.sort(np.array([tzmin, tzmax]).transpose())
    tzmin = ts[0]
    tzmax = ts[1]

    # if we miss the box in this plane, no collision
    isCollided = np.logical_and(isCollided, np.logical_not(np.logical_or((tmin > tzmax), (tzmin > tmax))))

    # identify the parameters to use with z
    tmin = np.maximum.reduce([tmin, tzmin])
    tmax = np.minimum.reduce([tmax, tzmax])

    # check that the intersecion is within the link length
    isCollided = np.logical_and(isCollided, np.logical_not(np.logical_or((0 > tmax), (1 < tmin))))
    isCollided = isCollided.reshape((isCollided.shape[0],1))
    return isCollided[0,0]
