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
        q_matrix = np.zeros((1, 6))
        # Your code starts from here
        
        q = q_matrix[0]

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

        # Rotation matrix from frame 3 to frame 4
        R_34 = np.zeros((3, 3))
        R_34[2,1] = -1
        R_34[1,2] = np.cos(q[3])
        R_34[1,0] = np.sin(q[3])
        R_34[0,2] = -np.sin(q[3])
        R_34[0,0] = np.cos(q[3])

        # Rotation matrix from frame 4 to frame 5
        R_45 = np.zeros((3, 3))
        R_45[2,2] = 1
        R_45[1,1] = np.cos(q[4])
        R_45[1,0] = np.sin(q[4])
        R_45[0,1] = -np.sin(q[4])
        R_45[0,0] = np.cos(q[4])

        # Positions
        x_wrist = 1
        y_wrist = 1
        z_wrist = 1

        # Theta_1
        theta1 = np.arctan2(y_wrist / x_wrist)

        # Theta_2
        theta2 = np.pi/2 - np.arctan2()

        # Rotation from frame 0 to frame 3 (wrist)
        R_03 = np.matmul(np.matmul(R_01, R_12), R_23)
        print("R_03 shape: ", R_03.shape) 
        # Rotation from frame 0 to end effector
        R = np.zeros((3, 3))
        for i in range(0, 3):
            for j in range(0, 3):
                R[i,j] = T0e[i,j]
        print("R shape w/ rows and cols: ", R.shape)
        # Rotation from wrist to end-effector
        R_3e = np.matmul(np.transpose(R_03), R)
        print('Wrist to end effector = ')
        print(R_3e)

        # Your code ends here

        return q_matrix, isPos
