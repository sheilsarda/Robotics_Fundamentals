import numpy as np
import math

class Main():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx ADL5 constants in mm
        self.d1 = 76.2                      # Distance between joint 1 and joint 2
        self.a2 = 146.05                    # Distance between joint 2 and joint 3
        self.a3 = 187.325                   # Distance between joint 3 and joint 4
        self.d4 = 34                        # Distance between joint 4 and joint 5
        self.d5 = 68                        # Distance between joint 4 and end effector

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def inverse(self, T0e):
        """
        INPUT:
        T - 4 x 4 homogeneous transformation matrix, representing
           the end effector frame expressed in the base (0) frame
           (position in mm)

        OUTPUT:
        q - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad)
           which are required for the Lynx robot to reach the given
           transformation matrix T. Each row represents a single
           solution to the IK problem. If the transform is
           infeasible, q should be all zeros.
        isPos - a boolean set to true if the provided
             transformation T is achievable by the Lynx robot as given,
             ignoring joint limits
        """
        isPos = 1
        q = np.zeros((1, 6))
        # Your code starts from here
        
        # Rotation matrix from frame 0 to frame 1
        R_01 = np.zeros((3, 3))
        R_01[2,1]= np.sin(q[0])
        R_01[1,2]= -np.cos(q[0])
        R_01[1,0]= np.sin(q[0])
        R_01[0,2]= -np.sin(q[0])
        R_01[0,0]= np.cos(q[0])

        # Rotation matrix from frame 1 to frame 2
        R_12 = np.zeros((3, 3))
        R_12[2,2]= 1
        R_12[1,1]= np.cos(q[1])
        R_12[1,0]= np.sin(q[1])
        R_12[0,1]= -np.sin(q[1])
        R_12[0,0]= np.cos(q[1])

        # Rotation matrix from frame 2 to frame 3
        R_23 = np.zeros((3, 3))
        R_23[2,2]= 1
        R_23[1,1]= np.cos(q[2])
        R_23[1,0]= np.sin(q[2])
        R_23[0,1]= -np.sin(q[2])
        R_23[0,0]= np.cos(q[2])

        # Rotation from frame 0 to frame 3 (wrist)
        R_03 = np.matmul(np.matmul(R_01, R_12), R_23)
        
        # Rotation from frame 0 to end effector
        R = T0e[0:2][0:2]

        # Rotation from wrist to end-effector
        R_3e = np.matmul(np.transpose(R_03), R)
        print('Wrist to end effector = ')
        print(R_3e)

        # Your code ends here

        return q, isPos
