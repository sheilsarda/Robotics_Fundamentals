import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
from copy import deepcopy
from time import sleep
from calculateFK import calculateFK

def obstacleCollision(start, goals, obstacles):
    """
    start is a pose containing q1 -> qe
    goals is a pose containing q1 -> qe
    returns if there is a feasible straight line path
    """
    lineCollision = False
    # print("================ Obstacle Collision ================")
    # print("lineCollision is " + str(lineCollision))
    # print("Start " + str(start))
    # print("Goal " + str(goals))

    f = calculateFK()

    # All the joint XYZ values
    startPos, _ = f.forward(start)

    # All the joint XYZ value
    goalPos, _ = f.forward(start)

    # Check if valid path exists between 
    # startPos[i] and goalPos[i]
    for obstacle in obstacles:
        results = detectCollision(startPos, goalPos, obstacle)
        for result in results:
            lineCollision |= result

    print("lineCollision is " + str(lineCollision))
    return lineCollision

def findNN(points, newPoint):
    """
    Return i such that points[i] is the closest distance
    from newPoint for all the points in the array
    """

    minI = -1
    minDist = 99999
    for i in range(len(points)):
        # find distance
        a = points[i]
        b = newPoint
        dist = np.abs(sum([(a[i] - b[i])**2 for i in range(3)])**0.5)
        
        # update minI
        if dist < minDist:
            minI = i
            minDist = dist

    return minI


def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (1x6).
    :param goal:        goal pose of the robot (1x6).
    :return:            returns an mx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix.
    """

    obstacles = map.obstacles
    boundary = map.boundary

    # print(obstacles)
    # print(boundary)

    # TODO - check if boundaries are inside of joint limits
    # in which case we need to update joint limits

    if (sum(goal - start) < 0.000001 ):
        print("start equals goal")
        # return [start]
    
    f = calculateFK()
    startPos, _ = f.forward(start)
    # startE = startPos[-1]
    
    goalPos, _ = f.forward(goal)
    # goalE = goalPos[-1]

    startE = [453, 0, -100]
    goalE = [-345, 50, 290]
    
    print("Start: ", startE)
    print("Goal: ", goalE)
    print("Obstacles: ", obstacles)

    # Check if straight line path exists between start and goal
    lineCollision = obstacleCollision([startE], [goalE], obstacles)
    # check if startE and goalE collides with itself
    startObstacle = obstacleCollision([startE], [startE], obstacles)
    goalObstacle = obstacleCollision([goalE], [goalE], obstacles)

    if not lineCollision:
        print ("no collision on straight line")
        # return [start, goal]
    elif(startObstacle or goalObstacle):
        print("Target or Start inside obstacle")
        # return ([])

    # currentPos
    currentPos = startE

    # list of feasible points on the line such that
    # feasible line exists between adjacent points
    # in the list
    points = [list(startE)]
    goalFound = False
    
    # total number of iterations 
    maxIter = 100
    
    i = 0

    self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
    self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    while (not goalFound and i < maxIter):
        # sample a pose
        #random thetas
        randQ1 = random.randrange(self.lowerLim[0], self.upperLim[0])
        randQ2 = random.randrange(self.lowerLim[1], self.upperLim[1])
        randQ3 = random.randrange(self.lowerLim[2], self.upperLim[2])
        randQ4 = random.randrange(self.lowerLim[3], self.upperLim[3])
        randQE = random.randrange(self.lowerLim[4], self.upperLim[4])
        
        newPose = [randQ1, randQ2, randQ3, randQ4, randQE]
        # print(i, newPose)

        coll = obstacleCollision([currentPos],[newPoint], obstacles)

        if(not coll and not obstacleCollision([newPoint],[goalE], obstacles)):
            points.append(list(newPoint))
            points.append(list(goalE))
            print("New Point", newPoint)
            print("Straight Line to goal feasible")
            goalFound = True

        elif not coll:
                points.append(list(newPoint))
                currentPos = newPoint
        i += 1
    print(len(points))


