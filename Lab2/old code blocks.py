

            # # mag. of x and y coord of wrist and end effector
            # a_squared = (wrist_pos[0]**2 + wrist_pos[1]**2 + wrist_pos[2]**2 )
            # l_squared = (e_pos[0]**2 + e_pos[1]**2 + e_pos[2]**2)

            # # mag. of vector between wrist and end effector
            # b_squared = 68

            # # angle between end-effector orientation and wrist orientation-- law of cosines
            # phi = np.arccos((l_squared - a_squared - b_squared) / (-2 * a_squared**0.5 * b_squared**0.5))
            
            # # alpha
            # alpha = np.pi - phi

            #        y_prime = T0e[0:3, 1] 

            # print("Intended y: ", y_prime)
 

            # T0e_feasible[0,2] = z_dot_x / z_norm
            # T0e_feasible[1,2] = z_dot_y / z_norm
            # T0e_feasible[2,2] = z_dot_z / z_norm

            # # Wrist position
            # wrist_pos = e_pos - self.d5*T0e_feasible[0:3,2] 


            # # Theta_1
            # theta1 = np.arctan(wrist_pos[1] / wrist_pos[0])
            # print('Theta_1 = ', theta1, ' rads')
            # q[0,0] = np.around(theta1, 5)
            # # Theta_3
            # theta3 = np.pi/2 -np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - self.d1)**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3))
            # q[0,2] = np.around(theta3, 5)
            # print('Theta_3 = ', q[0,2], ' rads')
            # # Theta_2
            # theta2 =  np.pi/2 - np.arctan2((wrist_pos[2] - self.d1) , (np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2))) + np.arctan2((self.a3*np.sin(-np.pi/2 - theta3)) , (self.a2 + self.a3*np.cos(-np.pi/2 - theta3))) 
            # q[0,1] = np.around(theta2, 5)
            # print('Theta_2 = ', q[0,1], ' rads')

            # i = 0

            # # y_prime dot R[0:4]
            # R_03  = np.matmul(np.matmul(R_01, R_12), R_23)
            # print("R_03 y", R_03)
            # y_vec = np.dot(R_03[0:3,2], y_prime) / np.linalg.norm(R_03[0:3,2])**2 * R_03[0:3,2]

            # y_norm = np.linalg.norm(y_vec)

            # T0e_feasible[0,1] = y_vec[0] / y_norm
            # T0e_feasible[1,1] = y_vec[1] / y_norm
            # T0e_feasible[2,1] = y_vec[2] / y_norm
            
            # print("dot with z0", np.dot(z_prime, [0, 0, 1]))
            # z0_vec = [z_prime[i] * np.dot(z_prime, [0, 0, 1]) for i in range(len(z_prime))]
            # print("dot with z0 mult", z0_vec)
            # z_e = z_e - z0_vec
            # z_e_norm = np.linalg.norm(z_e)
            # z_e = z_e / z_e_norm

            # print("Feasible z (2): ", z_e)