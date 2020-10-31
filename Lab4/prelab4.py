import numpy as np
from numpy import dot as dot

if __name__=='__main__':
    """
    This is the dimension of the Lynx Robot stated as global variable

    """

    # Zero pose
    q = [np.pi/2, 0, 0, 0, 0, 0]

    # Lynx Dimensions in mm
    L1 = 76.2    # distance between joint 0 and joint 1
    L2 = 146.05  # distance between joint 1 and joint 2
    L3 = 187.325 # distance between joint 2 and joint 3
    L4 = 34      # distance between joint 3 and joint 4
    L5 = 34      # distance between joint 4 and center of gripper

    # Joint limits
    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    # Frame 1 w.r.t Frame 0
    T1 = np.array([[np.cos(q[0]), -np.sin(q[0])*np.cos(-np.pi/2), np.sin(q[0])*np.sin(-np.pi/2), 0],
                    [np.sin(q[0]), np.cos(q[0])*np.cos(-np.pi/2), -np.cos(q[0])*np.sin(-np.pi/2), 0],
                    [0, np.sin(-np.pi/2), np.cos(-np.pi/2), L1],
                    [0, 0, 0, 1]])

    # Frame 2 w.r.t Frame 1
    T2 = np.array([[np.cos(q[1]-(np.pi/2)), -np.sin(q[1]-(np.pi/2)), 0, L2*np.cos(q[1]-(np.pi/2))],
                    [np.sin(q[1]-(np.pi/2)), np.cos(q[1]-(np.pi/2)), 0, L2*np.sin(q[1]-(np.pi/2))],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Frame 3 w.r.t Frame 2
    T3 = np.array([[np.cos(q[2]+(np.pi/2)), -np.sin(q[2]+(np.pi/2)), 0, L3*np.cos(q[2]+(np.pi/2))],
                    [np.sin(q[2]+(np.pi/2)), np.cos(q[2]+(np.pi/2)), 0, L3*np.sin(q[2]+(np.pi/2))],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Frame 4 w.r.t Frame 3
    T4 = np.array([[np.cos(q[3]-(np.pi/2)), -np.sin(q[3]-(np.pi/2))*np.cos(-np.pi/2), np.sin(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                    [np.sin(q[3]-(np.pi/2)), np.cos(q[3]-(np.pi/2))*np.cos(-np.pi/2), -np.cos(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                    [0, np.sin(-np.pi/2), np.cos(-np.pi/2), 0],
                    [0, 0, 0, 1]])
    # Frame 5 w.r.t Frame 4
    T5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, 0],
                    [np.sin(q[4]), np.cos(q[4]), 0, 0],
                    [0, 0, 1, L4 + L5],
                    [0, 0, 0, 1]])


    T01 = T1
    T02 = dot(T1, T2)
    T03 = dot(dot(T1, T2), T3)
    T04 = dot(dot(dot(T1, T2), T3), T4)
    T0e = dot(dot(dot(dot(T1, T2), T3), T4), T5)

    # Jacobian
    J = np.zeros((6, 5)).reshape((6, 5))

    # First three rows - skip first column since all zeros
    J[0:3, 1] = T01[0:3, 2].dot(T02[0:3, 3] - T01[0:3, 3])
    J[0:3, 2] = T02[0:3, 2].dot(T03[0:3, 3] - T02[0:3, 3])
    J[0:3, 3] = T03[0:3, 2].dot(T04[0:3, 3] - T03[0:3, 3])
    J[0:3, 4] = T04[0:3, 2].dot(T0e[0:3, 3] - T04[0:3, 3])

    # Bottom three rows - skip first column since all zeros
    J[3:6, 1] = T01[0:3, 2]
    J[3:6, 2] = T02[0:3, 2]
    J[3:6, 3] = T03[0:3, 2]
    J[3:6, 4] = T04[0:3, 2]

    print("Jacobian")
    print(J)
    print("----------------")





