import numpy as np

class Main():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx Dimensions in mm
        self.L1 = 76.2    # distance between joint 0 and joint 1
        self.L2 = 146.05  # distance between joint 1 and joint 2
        self.L3 = 187.325 # distance between joint 2 and joint 3
        self.L4 = 34      # distance between joint 3 and joint 4
        self.L5 = 34      # distance between joint 4 and center of gripper

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def forward(self, q):
        """
        INPUT:
        q - 1x6 vector of joint inputs [q0,q1,q2,q3,q4,lg]

        OUTPUTS:
        jointPositions - 6 x 3 matrix, where each row represents one
                  joint along the robot. Each row contains the [x,y,z]
                  coordinates of the respective joint's center (mm). For
                  consistency, the first joint should be located at
                  [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  base (0) frame
        """
        # Your code starts here

        jointPositions = np.zeros((6,3))
        T0e = np.identity(4)
        print("editing jointPositions")
        jointPositions[0,:]=0
        jointPositions[1,0:1]=0
        jointPositions[1,2]=self.L1
        jointPositions[2,0]=self.L2*np.sin(q[1])*np.cos(q[0])
        jointPositions[2,1]=self.L2*np.sin(q[1])*np.sin(q[0])
        jointPositions[2,2]=self.L2*np.cos(q[1])+self.L1
        jointPositions[3,0]=np.cos(q[0])*(self.L3*np.cos(q[2])+self.L2*np.sin(q[1]))
        jointPositions[3,1]=np.sin(q[0])*(self.L3*np.cos(q[2])+self.L2*np.sin(q[1]))
        jointPositions[3,2]=self.L2*np.cos(q[1])+self.L1-self.L3*np.sin(q[2])
        jointPositions[4,0]=np.cos(q[0])*(self.L3*np.cos(q[2])+self.L2*np.sin(q[1])+self.L4*np.cos(q[3]))
        jointPositions[4,1]=np.sin(q[0])*(self.L3*np.cos(q[2])+self.L2*np.sin(q[1])+self.L4*np.cos(q[3]))
        jointPositions[4,2]=self.L2*np.cos(q[1])+self.L1-self.L3*np.sin(q[2])-self.L4*np.sin(q[3])
        jointPositions[5,0]=np.cos(q[0])*(self.L3*np.cos(q[2])+self.L2*np.sin(q[1])+(self.L4+self.L5)*np.cos(q[3]))
        jointPositions[5,1]=np.sin(q[0])*(self.L3*np.cos(q[2])+self.L2*np.sin(q[1])+(self.L4+self.L5)*np.cos(q[3]))
        jointPositions[5,2]=self.L2*np.cos(q[1])+self.L1-self.L3*np.sin(q[2])-(self.L4+self.L5)*np.sin(q[3])
        
        print("Making the transformation matrix")
        # Rotation matrix from frame 0 to 1
        A_0_1 = np.zeros((4,4))
        A_0_1[3,3]=1
        A_0_1[2,2]=1
        A_0_1[1,1]=np.cos(q[0])
        A_0_1[1,0]=np.sin(q[0])
        A_0_1[0,1]=-np.sin(q[0])
        A_0_1[0,0]=np.cos(q[0])

        # Rotation matrix from frame 1 to 2
        A_1_2 = np.zeros((4,4))
        A_1_2[3,3]=1
        A_1_2[2,3]=self.L1
        A_1_2[2,1]=-1
        A_1_2[1,2]=np.cos(np.pi - q[1])
        A_1_2[1,0]=np.sin(np.pi - q[1])
        A_1_2[0,2]=-np.sin(np.pi - q[1])
        A_1_2[0,0]=np.cos(np.pi - q[1])

	# Rotation matrix from frame 2 to 3
        A_2_3 = np.zeros((4,4))
        A_2_3[3,3]=1
        A_2_3[2,2]=1
        A_2_3[1,3]=self.L2*np.sin(q[2] + np.pi/2)
        A_2_3[1,1]=np.cos(q[2] + np.pi/2)
        A_2_3[1,0]=np.sin(q[2] + np.pi/2)
        A_2_3[0,3]=self.L2*np.cos(q[2] + np.pi/2)
        A_2_3[0,1]=-np.sin(q[2] + np.pi/2)
        A_2_3[0,0]=np.cos(q[2] + np.pi/2)

        # Rotation matrix from frame 3 to 4
        A_3_4 = np.zeros((4,4))
        A_3_4[3,3]=1
        A_3_4[2,2]=1
        A_3_4[1,3]=self.L3*np.sin(q[3]-np.pi/2)
        A_3_4[1,1]=np.cos(q[3]-np.pi/2)
        A_3_4[1,0]=np.sin(q[3]-np.pi/2)
        A_3_4[0,0]=np.cos(q[3]-np.pi/2)
        A_3_4[0,1]=-np.sin(q[3]-np.pi/2)
        A_3_4[0,1]=self.L3*np.cos(q[3]-np.pi/2)


        # Rotation matrix from frame 4 to 5
        A_4_5 = np.zeros((4,4))
        A_4_5[3,3]=1
        A_4_5[2,1]=-1
        A_4_5[1,0]=1
        A_4_5[0,2]=-1

        # Rotation matrix from frame 5 to e
        A_5_e = np.zeros((4,4))
        A_5_e[3,3]=1
        A_5_e[2,3]=self.L4 + self.L5
        A_5_e[2,2]=1
        A_5_e[1,0]=1
        A_5_e[0,1]=-1

        # T0e = (A_0_1 * A_1_2 * A_2_3 * A_3_4 * A_4_5 * A_5_e)
	T0e = np.linalg.multi_dot([A_0_1, A_1_2, A_2_3, A_3_4, A_4_5, A_5_e])
	# T0e = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2), A_2_3), A_3_4), A_4_5), A_5_e)
        # Your code ends here

        return jointPositions, T0e
