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

    #constants to control
    zeta=2
    eta=0.0001
    rho0=0.0001
    alpha=0.015
    
    #intializing vectors
    Fatt=np.zeros((6, 3))
    Frep=np.zeros((6, 3))
    Fnet=np.zeros((6, 3))
    tor=np.zeros(6)
    
    
    R=calculateFK()
    Jp,T0e=R.forward(qCurr)
    Jpg,T0eg=R.forward(qGoal)
    #print("current position:",Jp[3])
    #print("goal position:",Jpg[3])
    #print("distance away:\n",Jp-Jpg)

    rho = []
    unit=[]
    for box in map_struct.obstacles:
        dist_i, unit_i = distPointToBox(Jp, box)
        rho.append(dist_i)
        unit.append(unit_i)

    
    for i in range(0,6):
        # Compute force of repulsion
        for j in range(len(rho)):
            # distance between joint and current obstacle
            drho=unit_i[j]
            # Add current obstacle's repulsive force to accumulated repulsive force on joint
            Frep[i] += eta*(1/rho[j][i] - 1/rho0)*(1/rho[j][i]**2)*drho
            
        Fatt[i] = -zeta*(Jp[i] - Jpg[i])
        Fnet[i] = Fatt[i] + Frep[i]
    #print("Fnet:",Fnet)
        
    #USING JACOBIAN:
    Jac1=np.zeros(3).reshape((3,1))
    Jac=calcJacobian(qCurr,6)[:3]
    Jac=np.hstack((Jac,Jac1))
    tori = np.matmul(Jac.T,Fnet.T)
    tor=np.sum(tori,axis=1)
    unit_tor=(tor)/np.linalg.norm(tor)
    unit_tor=np.nan_to_num(unit_tor,False,0)
    #unit_tor=np.append(unit_tor,[0])
    qNext = np.around(qCurr+alpha*(unit_tor),4)
    
    #Updating isDone
#     done=0
#     for r in range(0,3):
# #         print(abs(qNext[r]-qGoal[r]))
#         if abs(qNext[r]-qGoal[r])<0.05:
# #             print("joint value")
#             done+=1
#     print(done)

#     if done==3:
#         isDone=True
#     else:
#         isDone=False
    
    return qNext