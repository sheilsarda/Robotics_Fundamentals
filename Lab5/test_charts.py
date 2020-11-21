from potentialFieldPath import potentialFieldPath

import numpy as np
from copy import deepcopy
import sys
import random
from sys import path
from calculateFK import calculateFK
from copy import deepcopy
from time import sleep
import numpy as np
import sys
from random import random as rand
from loadmap import loadmap

from matplotlib import pyplot as plt
import random
from sys import path
from os import getcwd
from calculateFK import calculateFK
from potentialFieldStep import potentialFieldStep

if __name__=='__main__':
    # Update map location with the location of the target map
    map_struct = loadmap("maps/map3.txt")


    #start = np.array([0, 0, 0, 0, 0, 0])
    #end = np.array([0.5, -0.5, -1.4, -1, 0, 0])

    start = [0,  0, 0, 0, 0, 0]
    end = [1.4, 0, 1, 0, 0, 0]


    path1 = potentialFieldPath(map_struct, start, end)
    #print(path1)
    
    
    #ig1, axs1 = plt.subplots(nrows=2, ncols=3)
    # plt.rcParams['figure.figsize'] = (5.0, 4.0)

    #um_points = len(path1)

    #rint("length of path array " + str(num_points))
    #rint(path1[-1])

    #l=np.array(range(0,num_points))
    #=np.empty(num_points)
    #or i in range(6):
    #   for l in range(len(xl) - num_points, len(xl) ):
    #       y[l]=path1[l][i]
    #   axs1[i%2, i%3].plot(xl.copy(), y.copy())
    #   axs1[i%2, i%3].set_title("Joint " + str(i))
   #    y = np.empty(num_points)
        
   #plt.show()
