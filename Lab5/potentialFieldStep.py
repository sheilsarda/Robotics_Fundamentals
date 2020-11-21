from copy import deepcopy
import numpy as np
from calculateFK import calculateFK
from distPointToBox import distPointToBox
from calcJacobian import calcJacobian
import math

def potentialFieldStep(qCurr, map, qGoal, params):
    """
    This function plans a path through the map using a potential field planner
    :param qCurr:       current pose of the robot (1x6).
    :param map:         the map struct
    :param qGoal:       goal pose of the robot (1x6).
    :return:
        qNext - 1x6 vector of the robots next configuration
        isDone - TRUE: when the robot has reached the goal or is stuck
                 FALSE: otherwise
    """

    #constants to control
    eta = params[0] 
    zeta = params[1] 
    rho0 = params[2]
    alpha = params[3] 

    delnan=False

    #intializing vectors
    Fatt=np.zeros((6, 3))
    Frep=np.zeros((6, 3))
    Fnet=np.zeros((6, 3))
    tor=np.zeros(6)


    R=calculateFK()
    Jp,T0e=R.forward(qCurr)
    Jpg,T0eg=R.forward(qGoal)

    rho = []
    unit=[]
    for box in map.obstacles:
        dist_i, unit_i = distPointToBox(Jp, box)
        rho.append(dist_i)
        unit.append(unit_i)


    for i in range(0,6):
        # Compute force of repulsion
        for j in range(len(rho)):
            # distance between joint and current obstacle
            drho=unit_i[j]
            # Add current obstacle's repulsive force to accumulated repulsive force on joint
            if(rho[j][i] <= rho0):
                Frep[i] += eta*(1/rho[j][i] - 1/rho0)*(1/rho[j][i]**2)*drho

        Fatt[i] = -zeta*(Jp[i] - Jpg[i])
        Fnet[i] = Fatt[i] + Frep[i]

    #USING JACOBIAN:
    Jac1=np.zeros(3).reshape((3,1))
    Jac=calcJacobian(qCurr,6)[:3]
    Jac=np.hstack((Jac,Jac1))
    tori = np.matmul(Jac.T,Fnet.T)
    tor=np.sum(tori,axis=1)
    unit_tor=(tor)/np.linalg.norm(tor)
    
    for l in range(0,6):
        if np.isnan(tor[l]):
            delnan=True
    
    qNext = np.around(qCurr+alpha*(unit_tor),4)

    lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]
    upperLim = [1.4, 1.4, 1.7, 1.7, 1.5, 30]

    qFiltered = deepcopy(qNext)
    for i in range(len(qFiltered)):
        qFiltered[i] = math.fmod(qFiltered[i], 2*np.pi)
        if qFiltered[i] > upperLim[i]:
            qFiltered[i] = upperLim[i]
        elif qFiltered[i] < lowerLim[i]:
            qFiltered[i] = lowerLim[i]

    return qFiltered,delnan
