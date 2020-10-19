import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
from copy import deepcopy
from time import sleep
from calculateFK import calculateFK

def obstacleCollision(start, goals, obstacles):
    lineCollision = False
    # print("================ Obstacle Collision ================")
    # print("lineCollision is " + str(lineCollision))
    # print("Start " + str(start))
    # print("Goal " + str(goals))

    for obstacle in obstacles:
        results = detectCollision(start, goals, obstacle)
        lineCollision |= results[0]
        print(obstacle, results)

    # print("lineCollision is " + str(lineCollision))
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

    obstacles = map_struct.obstacles
    boundary = map_struct.boundary

    # print(obstacles)
    # print(boundary)

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

    while (not goalFound and i < maxIter):
        # sample a point
        #random x integer generated from boundary min and max x
        randX = random.randrange(boundary[0], boundary[3])
        #random y integer generated from boundary min and max y
        randY = random.randrange(boundary[1], boundary[4])
        #random z integer generated from boundary min and max z
        randZ = random.randrange(boundary[2], boundary[5])
        
        #use current position and new rand x,y,z to see if collisions exists
        newPoint = [randX, randY, randZ]
        # print(i, newPoint)

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


