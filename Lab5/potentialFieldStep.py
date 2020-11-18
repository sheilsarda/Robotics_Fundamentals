import numpy as np
from calculateFK import calculateFK

def potentialFieldStep(qCurr, map, qGoal):
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

    zeta = 0
    eta = 0
    rho0 = 0

    R = calculateFK()
    Jp, T0e = R.forward(qCurr)

    Jpg, T0g = R.forward(qGoal)
    
    Fatt = np.zeros(6)
    Fatt = np.zeros(6)

    for i in range(0, 6):
        
        Fatt[i] = -zeta * (Jp[i] - Jpg[i])
        drho = (Jp[i] - b) / 
        Frep[i] = eta*(1 / (rho * Jp(qCurr)[i]) - 1/rho0) * \
                    (1 / (rho^2 * (Jp(qCurr)[i])))*drho
        
        Fnet[i] = Fatt[i] + Frep[i]
        tor[i] = np.matmul(calcJacobian(qCurr, i).T, Fnet[i])
        qNext = qCurr + alpha * (tor) / np.linalg.norm(tor)

    if abs(qNext - qCurr)< 0.005:
        isDone = True
    else:
        isDone = False
    
    return qNext, isDone