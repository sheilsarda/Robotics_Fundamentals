import numpy as np
from potentialFieldStep import potentialFieldStep
from copy import deepcopy
import sys
from random import random as rand
from sys import path
from os import getcwd

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
    #need to define these variables
    
    thresh = 0.04
    
    ipath=[]
    path=[]
    isDoneArray = []
    qNext=potentialFieldStep(qStart,map_struct,qGoal)
    qNext[4]=qGoal[4]
    qNext[5]=qGoal[5]
    path.append(qNext)
    ipath.append(qNext)
    #isDoneArray.append(isDone)
    print("1st qCurr:",qNext)
    f=0
    isDone=False
    while isDone==False and f<=5000:
        f+=1
        qCurr=ipath[-1]
        qNext=potentialFieldStep(qCurr,map_struct,qGoal)
        ipath.append(qNext)
        done=0
        ind=0
        #print(ipath[-1][2])
        
        if(f%1000==0):
            print("iteration #:",f)
        
        if (f%100)==0:    
            path.append(qNext)
            if f>=1000:
                #IS DONE CHECK
                for r in range(0,3):
                    # print("how close we are to converging:",abs(qNext[r]-path[-4][r]))
                    if abs(qNext[r]-path[-4][r])<thresh:
                        done+=1
                print("done value:",done)
        
        #If isdone is true, We will check whether it reached its goal:
        if done==3:
            isDone=True
        g=0
        for r in range(0,3):
            #print("close to goal:",abs(qGoal[r]-path[-1][r]))
            if abs(qGoal[r]-path[-1][r])<thresh:
                g+=1
        if g==3:
            print("goal reached")
            isDone = True
    
    print("IsDone", isDone)
    print("g", g)
    if g != 3:
        
        numPoints = 1
        while(numPoints <= 10):
            
            print("numPoints", numPoints)
            
            print("stuck at minima. Initiating RRT")
            points = rrt(map_struct, path[-1], numPoints)
            print("Done rrt: ", points)

            rrt_goal=0
            for r in range(0,3):
                print("close to goal:",abs(points[-1][r]-path[-1][r]))
                if abs(points[-1][r]-path[-1][r])<thresh:
                    rrt_goal+=1
            if rrt_goal==3:
                print("goal reached through rrt")                
                path2 = potentialFieldPath(map_struct, points[-1], qGoal)
                path.append(points)
                path.append(path2)
                return (path) 
            numPoints+=1
        print("RRT was not able to reach the goal in numPoints")
    return path

def boundaryCollision(point, boundary):
    outOfBounds = False
    # print("================ Boundary Collision ================")
    # print("Point " + str(point))

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
    # print("================ Obstacle Collision ================")
    # print("lineCollision is " + str(lineCollision))
    # print("Start " + str(start))
    # print("Goal " + str(goal))

    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(start[0])
    goalPos, _ = f.forward(goal[0])

    # print("StartPos " + str(startPos))
    # print("GoalPos " + str(goalPos))

    for obstacle in obstacles:
        # Iterate through every joint
        for i in range(len(startPos)):
            results = detectCollision([startPos[i]], [goalPos[i]], obstacle)
            for result in results:
                lineCollision |= result

    # print("lineCollision is " + str(lineCollision))
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
            # print("Eliminated %i elements"%(b-a - 1))

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
    bufferRadius = 40

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
    points = [list(start)]
    maxIter = numPoints
    i = 0

    # Lower joint limits in radians (grip in mm
    # (negative closes more firmly))
    lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]

    # Upper joint limits in radians (grip in mm)
    upperLim = [1.4, 1.4, 1.7, 1.7, 1.5, 30]
    
    currentPose = start
    
    print("(1) Above RRT While")

    while (i < maxIter):
        # sample a pose
        randQ1 = random.uniform(lowerLim[0], upperLim[0])
        randQ2 = random.uniform(lowerLim[1], upperLim[1])
        randQ3 = random.uniform(lowerLim[2], upperLim[2])
        randQ4 = random.uniform(lowerLim[3], upperLim[3])
        randQE = random.uniform(lowerLim[4], upperLim[4])
        
        print("(2.1) Generated RRT point")

        newPose = [randQ1, randQ2, randQ3, randQ4, randQE, 0]
        #print(i, newPose)

        coll = obstacleCollision([currentPose],[newPose], obstacles)
        coll |= boundaryCollision([currentPose], boundary)

        if not coll:
            points.append(list(newPose))
            currentPose = newPose
            i += 1
        if i==maxIter:
            print("max iterations reached")
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

    # %% Begin Collision Detection

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