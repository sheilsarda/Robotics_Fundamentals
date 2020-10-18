import numpy as np
import math
import copy

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

        # End effector position
        e_pos = T0e[0:3,3]

        # Wrist position
        wrist_pos = e_pos - self.d5*T0e[0:3,2]
        print("wrist position: ", wrist_pos)
        print("end_effector position: ", e_pos)

        # Theta_1
        theta1 = np.arctan2(wrist_pos[1] , wrist_pos[0])
        print('Theta_1 = ', theta1, ' rads')
        q[0,0] = np.around(theta1, 5)

        # Check for feasible orientation of end-effector
        theta_check = np.arctan2(e_pos[1], e_pos[0])
        if(abs(theta_check-theta1) <  0.000001):
            isPos = True
        else: isPos = False

        print("isPossible: " , isPos)

        # Theta_3
        theta3 = np.pi/2 -np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - self.d1)**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3))
        q[0,2] = np.around(theta3, 5)
        print('Theta_3 = ', q[0,2], ' rads')

        # Theta_2
        theta2 =  np.pi/2 - np.arctan2((wrist_pos[2] - self.d1) , (np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2))) + np.arctan2((self.a3*np.sin(-np.pi/2 - theta3)) , (self.a2 + self.a3*np.cos(-np.pi/2 - theta3)))
        q[0,1] = np.around(theta2, 5)
        print('Theta_2 = ', q[0,1], ' rads')

        print('q before populating every theta: ', q)

        # Rotation matrix from frame 0 to frame 1
        R_01 = np.zeros((3, 3))
        R_01[2,1]= -1
        R_01[1,2]= np.cos(q[0,0])
        R_01[1,0]= np.sin(q[0,0])
        R_01[0,2]= -np.sin(q[0,0])
        R_01[0,0]= np.cos(q[0,0])

        # Rotation matrix from frame 1 to frame 2
        R_12 = np.zeros((3, 3))
        R_12[2,2]= 1
        R_12[1,1]= np.cos(q[0,1]-np.pi/2)
        R_12[1,0]= np.sin(q[0,1]-np.pi/2)
        R_12[0,1]= -np.sin(q[0,1]-np.pi/2)
        R_12[0,0]= np.cos(q[0,1]-np.pi/2)

        # Rotation matrix from frame 2 to frame 3
        R_23 = np.zeros((3, 3))
        R_23[2,2]= 1
        R_23[1,1]= np.cos(q[0,2]+np.pi/2)
        R_23[1,0]= np.sin(q[0,2]+np.pi/2)
        R_23[0,1]= -np.sin(q[0,2]+np.pi/2)
        R_23[0,0]= np.cos(q[0,2]+np.pi/2)

        # Rotation matrix from frame 3 to frame 4
        R_34 = np.zeros((3, 3))
        R_34[2,1] = -1
        R_34[1,2] = np.cos(q[0,3]-np.pi/2)
        R_34[1,0] = np.sin(q[0,3]-np.pi/2)
        R_34[0,2] = -np.sin(q[0,3]-np.pi/2)
        R_34[0,0] = np.cos(q[0,3]-np.pi/2)

        # Rotation matrix from frame 4 to frame 5
        R_45 = np.zeros((3, 3))
        R_45[2,2] = 1
        R_45[1,1] = np.cos(q[0,4])
        R_45[1,0] = np.sin(q[0,4])
        R_45[0,1] = -np.sin(q[0,4])
        R_45[0,0] = np.cos(q[0,4])

        # Rotation from frame 0 to end effector
        R = np.zeros((3, 3))
        for i in range(0, 3):
            for j in range(0, 3):
                R[i,j] = T0e[i,j]
        print('R_0e = ')
        print(R)

        if(isPos):
            # Theta_4
            theta4 = np.arccos(T0e[0,2] / np.cos(q[0,0])) - q[0,1] - q[0,3]
            # print('Theta_4 = ', theta4, ' rads')
            q[0,3] = theta4

            # Theta_5
            R_03 = np.matmul(np.matmul(R_01, R_12), R_23)
            R_04 = np.matmul(R_03, R_34)
            # print('R_04 = ')
            # print(R_04)

            R_45_calc = np.matmul(np.transpose(R_04), R)
            # print('R_45_calc = ')
            # print(R_45_calc)

            theta5 = np.arccos(R_45_calc[0,0])
            # print('Theta_5 = ', theta5, ' rads')
            q[0,4] = theta5

            print('q after populating every theta: ', q)

        else:
            # Orientation of end effector
            # is unreachable. Goal is to
            # project z-orientation

            # mag. of unachievable orientation
            z_prime = T0e[0:3, 2]
            z_prime_mag = np.linalg.norm(z_prime)

            y_prime = T0e[0:3, 1]
            y_prime_mag = np.linalg.norm(y_prime)

            print("Intended y: ", y_prime)
            print("Intended z: ", z_prime)

            # mag. of x and y coord of wrist
            a_squared = (wrist_pos[0]**2 + wrist_pos[1]**2 + wrist_pos[2]**2 )

            # mag. of x and y coord of end effector
            l_squared = (e_pos[0]**2 + e_pos[1]**2 + e_pos[2]**2)

            # mag. of vector between wrist and end effector
            b_squared = 68*(T0e[0,2]**2 + T0e[1,2]**2 + T0e[2,2]**2)

            # angle between end-effector orientation
            # and wrist orientation-- law of cosines
            phi = np.arccos((l_squared - a_squared - b_squared) / (-2 * a_squared**0.5 * b_squared**0.5))

            # alpha
            alpha = np.pi - phi

            # z projection
            z_proj = z_prime_mag * (1 - np.cos(alpha))
            z_proj_mag = np.linalg.norm(z_proj)
            z_feasible = (z_proj / z_proj_mag)

            # y projection
            y_proj = y_prime_mag * (1 - np.cos(alpha))
            y_proj_mag = np.linalg.norm(y_proj)
            y_feasible = (y_proj / y_proj_mag)

            x_feasible = np.cross(y_feasible, z_feasible)

            print("Feasible x: ", x_feasible)
            print("Feasible y: ", y_feasible)
            print("Feasible z: ", z_feasible)


            T0e_feasible = copy.deepcopy(T0e)

            # X feasible
            # T0e_feasible[0,0] =



        # Your code ends here

        return q, isPos
