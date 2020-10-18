
import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
from astar import Astar
from copy import deepcopy
from time import sleep
from calculateFK import calculateFK

if __name__=='__main__':
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (1x6).
    :param goal:        goal pose of the robot (1x6).
    :return:            returns an mx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix..
    """

    map_struct = loadmap("maps/testMap.txt")
    start = np.array([0,  0, 0, 0, 0, 0])
    goal = np.array([0, 0, 1.1, 0, 0, 0])
    # path = Astar(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    # print(path)
    
    obstacles = map_struct.obstacles
    boundary = map_struct.boundary

    print(obstacles)
    print(boundary)

    if (sum(goal - start) < 0.000001 ):
        print("start equals goal")
        # return [start]
    
    f = calculateFK()
    startPos, _ = f.forward(start)
    startE = startPos[-1]
    
    goalPos, _ = f.forward(goal)
    goalE = goalPos[-1]
    
    print(startPos, goalPos)

    lineCollision = False
    for obstacle in obstacles:
        results = detectCollision([startE], [goalE], obstacle)
        lineCollision &= results[0]
        print(results)
    
    if not lineCollision:
        print ("no collision on straight line")
        # return [start, goal]
    


    