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

        # Positions -- dummy values for now
        x_wrist = 200
        y_wrist = 200
        z_wrist = 1

        # Theta_1
        theta1 = np.arctan2(y_wrist , x_wrist)
        print('Theta_1 = ', theta1, ' rads')
        q[0,0] = theta1

        # Theta_3
        theta3 = -np.pi/2 - np.arccos((x_wrist**2 + y_wrist**2 + (z_wrist - self.d1**2) - self.a2**2 - self.a3**2) / (2*self.a2*self.a3))
        print('Theta_3 = ', theta3, ' rads')
        q[0,2] = theta3

        # Theta_2
        theta2 =  np.pi/2 - np.arctan2((z_wrist - self.d1) , (np.sqrt(x_wrist**2 + y_wrist**2))) + np.arctan2((self.a3*np.sin(-np.pi/2 - theta3)) , (self.a2 + self.a3*np.cos(-np.pi/2 - theta3))) 
        print('Theta_2 = ', theta2, ' rads')
        q[0,1] = theta2

        print('q before populating every theta: ', q)


        # Rotation matrix from frame 0 to frame 1
        R_01 = np.zeros((3, 3))
        R_01[2,1]= np.sin(q[0,0])
        R_01[1,2]= -np.cos(q[0,0])
        R_01[1,0]= np.sin(q[0,0])
        R_01[0,2]= -np.sin(q[0,0])
        R_01[0,0]= np.cos(q[0,0])

        # Rotation matrix from frame 1 to frame 2
        R_12 = np.zeros((3, 3))
        R_12[2,2]= 1
        R_12[1,1]= np.cos(q[0,1])
        R_12[1,0]= np.sin(q[0,1])
        R_12[0,1]= -np.sin(q[0,1])
        R_12[0,0]= np.cos(q[0,1])

        # Rotation matrix from frame 2 to frame 3
        R_23 = np.zeros((3, 3))
        R_23[2,2]= 1
        R_23[1,1]= np.cos(q[0,2])
        R_23[1,0]= np.sin(q[0,2])
        R_23[0,1]= -np.sin(q[0,2])
        R_23[0,0]= np.cos(q[0,2])

        # Rotation matrix from frame 3 to frame 4
        R_34 = np.zeros((3, 3))
        R_34[2,1] = -1
        R_34[1,2] = np.cos(q[0,3])
        R_34[1,0] = np.sin(q[0,3])
        R_34[0,2] = -np.sin(q[0,3])
        R_34[0,0] = np.cos(q[0,3])

        # Rotation matrix from frame 4 to frame 5
        R_45 = np.zeros((3, 3))
        R_45[2,2] = 1
        R_45[1,1] = np.cos(q[0,4])
        R_45[1,0] = np.sin(q[0,4])
        R_45[0,1] = -np.sin(q[0,4])
        R_45[0,0] = np.cos(q[0,4])

        # Rotation from frame 0 to frame 3 (wrist) using post-multiplication of DH
        R_03 = np.matmul(np.matmul(R_01, R_12), R_23)

        # Rotation from frame 3 (wrist) to frame e (end-effector) using post-multiplication of DH
        R_3e = np.matmul(R_34, R_45)
        
        # Rotation from frame 0 to end effector
        # R = np.zeros((3, 3))
        # for i in range(0, 3):
        #     for j in range(0, 3):
        #         R[i,j] = T0e[i,j]
        
        # Input a rotation matrix to find theta4 and theta5
        R = np.eye((3))
        R[0,0] = -1
        R[1,1] = -1

        R_3e_test = np.matmul(np.linalg.inv(R_03), R)
        print('R_3e = ')
        print(R_3e_test)

        # Theta_5
        theta5 = np.arccos(-R_3e[2,1])
        print('Theta_5 = ', theta5, ' rads')
        q[0,4] = theta5
        
        # Theta_4
        theta4 = np.arccos(R_3e[0,0] / np.cos(theta5))
        print('Theta_4 = ', theta4, ' rads')
        q[0,3] = theta4

        # R_3e_check
        print('R_3e from DH = ')
        print(R_3e)        

        print('q after populating every theta: ', q)
        # Your code ends here

        return q_matrix, isPos
