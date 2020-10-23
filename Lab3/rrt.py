import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
from copy import deepcopy
from time import sleep
from calculateFK import calculateFK
from matplotlib import pyplot as plt

def inverse(T0e):
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

          # Lynx ADL5 constants in mm
        d1 = 76.2                      # Distance between joint 1 and joint 2
        a2 = 146.05                    # Distance between joint 2 and joint 3
        a3 = 187.325                   # Distance between joint 3 and joint 4
        d4 = 34                        # Distance between joint 4 and joint 5
        d5 = 68                        # Distance between joint 4 and end effector

        # Joint limits
        lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)


        isPos = 1
        q = np.zeros((1, 6))
        # Your code starts from here

        # End effector position
        e_pos = T0e[0:3,3]

        # Wrist position
        wrist_pos = e_pos - d5*T0e[0:3,2]
        print("wrist position: ", wrist_pos)
        print("end_effector position: ", e_pos)

        print("Target T0e: ", T0e)

        # Theta_1
        # First and Fourth quadrant
        theta1 = np.arctan2(wrist_pos[1],wrist_pos[0])
        print('Theta_1 = ', theta1, ' rads')
        q[0,0] = np.around(theta1, 5)

        # Second quadrant and Third quadrant yield the same solutions
        # as First and Fourth

        # Check for feasible orientation of end-effector
        theta_check = np.arctan2(e_pos[1],e_pos[0])
        if(abs(theta_check-theta1) <  0.0001):
            isPos = True
        else: isPos = False

        # Hardcoded
        print("isPossible: " , isPos)


        # Theta_3-- first solution (ELBOW UP)
        theta3 = -np.pi/2 +np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - d1)**2 - a2**2 - a3**2) / (2*a2*a3))
        q[0,2] = np.around(theta3, 5)
        print('Theta_3 = ', q[0,2], ' rads')
        #theta3 = -np.pi/2 -np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - self.d1)**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3))
        #q[0,2] = np.around(theta3, 5)
        #print('Theta_3 = ', q[0,2], ' rads')
        new_row = np.zeros((1,6))
        new_row[0,0] = theta1+np.pi
        new_row[0,2] = theta3
        q = np.concatenate((q, new_row))
        print('Theta_3 = ', new_row, ' rads')
        # Theta_3-- second solution
        theta3_2 = -np.pi/2 -np.arccos((wrist_pos[0]**2 + wrist_pos[1]**2 + (wrist_pos[2] - d1)**2 - a2**2 - a3**2) / (2*a2*a3))
        #q = np.concatenate((q, new_row))
        #print('Theta_3 = ', temp, ' rads')
        if(theta3_2 != None):
            new1_row = np.zeros((1,6))
            new1_row[0,0] = np.pi-theta1
            new1_row[0,2] = theta3_2
            q = np.concatenate((q, new1_row))
            new2_row = np.zeros((1,6))
            new2_row[0,0] = theta1
            new2_row[0,2] = theta3_2
            q = np.concatenate((q, new2_row))

        # Theta_2
        for i in range(q.shape[0]):
            theta3 = q[i, 2]
            theta2 =  np.pi/2 - np.arctan2((wrist_pos[2] - d1) , (np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2))) + np.arctan2((a3*np.sin(-np.pi/2 - theta3)) , (a2 + a3*np.cos(-np.pi/2 - theta3)))
            q[i,1] = np.around(theta2, 5)
            print('Theta_2 = ', q[i,1], ' rads')

        print('q before populating every theta: ', q)

        # row index of q array for rotational matrices to reference
        i = 0

        # Rotation matrix from frame 0 to frame 1
        R_01 = np.zeros((3, 3))
        R_01[2,1]= -1
        R_01[1,2]= np.cos(q[i,0])
        R_01[1,0]= np.sin(q[i,0])
        R_01[0,2]= -np.sin(q[i,0])
        R_01[0,0]= np.cos(q[i,0])

        # Rotation matrix from frame 1 to frame 2
        R_12 = np.zeros((3, 3))
        R_12[2,2]= 1
        R_12[1,1]= np.cos(q[i,1]-np.pi/2)
        R_12[1,0]= np.sin(q[i,1]-np.pi/2)
        R_12[0,1]= -np.sin(q[i,1]-np.pi/2)
        R_12[0,0]= np.cos(q[i,1]-np.pi/2)

        # Rotation matrix from frame 2 to frame 3
        R_23 = np.zeros((3, 3))
        R_23[2,2]= 1
        R_23[1,1]= np.cos(q[i,2]+np.pi/2)
        R_23[1,0]= np.sin(q[i,2]+np.pi/2)
        R_23[0,1]= -np.sin(q[i,2]+np.pi/2)
        R_23[0,0]= np.cos(q[i,2]+np.pi/2)

        # Rotation matrix from frame 3 to frame 4
        R_34 = np.zeros((3, 3))
        R_34[2,1] = -1
        R_34[1,2] = np.cos(q[i,3]-np.pi/2)
        R_34[1,0] = np.sin(q[i,3]-np.pi/2)
        R_34[0,2] = -np.sin(q[i,3]-np.pi/2)
        R_34[0,0] = np.cos(q[i,3]-np.pi/2)

        # Rotation matrix from frame 4 to frame 5
        R_45 = np.zeros((3, 3))
        R_45[2,2] = 1
        R_45[1,1] = np.cos(q[i,4])
        R_45[1,0] = np.sin(q[i,4])
        R_45[0,1] = -np.sin(q[i,4])
        R_45[0,0] = np.cos(q[i,4])

        # Rotation from frame 0 to end effector
        R = np.zeros((3, 3))
        for i in range(0, 3):
            for j in range(0, 3):
                R[i,j] = T0e[i,j]

        print('R_0e = ', R)

        if(isPos):
            for row_idx in range(q.shape[0]):
                # Which row from q to reference for matrix angles
                print("Calculating theta4 and theta5 for row ", i, " of ", q.shape[0])
                i = row_idx

                R_03 = np.matmul(np.matmul(R_01, R_12), R_23)
                print("R_03 ")
                print(R_03)

                R_3e = np.matmul(np.transpose(R_03) , R)
                print("R_3e ")
                print(R_3e)

                theta5 = np.arctan2(-R_3e[2,0] , -R_3e[2,1])
                theta4 = np.arctan2(-R_3e[0,2] , R_3e[1,2]) + np.pi/2

                print('Theta_4 = ', theta4, ' rads')
                print('Theta_5 = ', theta5, ' rads')

                q[i,3] = theta4
                q[i,4] = theta5

                R_check = np.matmul(np.matmul(R_03, R_34), R_45)
                print("R_check", R_check)

            print('q after populating every theta: ', q)

        else:
            T0e_feasible = copy.deepcopy(T0e)
            # Orientation of end effector
            # is unreachable. Goal is to
            # project z-orientation

            print("end effector", e_pos)
            print("wrist ", wrist_pos)
            theta1 = np.arctan2(e_pos[1],e_pos[0])
            print("theta1", theta1)
            theta1_w = np.arctan2(wrist_pos[1],wrist_pos[0])
            print("theta1_w", theta1_w)

            normal_vec = [-np.sin(theta1), np.cos(theta1), 0]
            z_prime = T0e[0:3, 2]
            z_prime_norm = np.linalg.norm(z_prime)

            dot_value = np.dot(z_prime, normal_vec)
            normal_dot = [normal_vec[i] * dot_value for i in range(len(normal_vec))]
            norm_normal_dot = np.linalg.norm(normal_vec)**2

            z_e_1 = (copy.deepcopy(z_prime) / z_prime_norm) - (normal_dot / norm_normal_dot)
            z_e_norm = np.linalg.norm(z_e_1)
            z_e = z_e_1 / z_e_norm

            print("normal_vec", normal_vec)
            print("z_prime", z_prime)
            print("dot prod norm", norm_normal_dot)
            print("normalized dot product", (normal_dot / norm_normal_dot))
            print("z_prime", z_prime)
            print("normal_dot", normal_dot)
            print("dot_value", dot_value)

            print("normal_vec", normal_vec)
            print("Feasible z (non-normalized): ", z_e)

            print("Feasible z (1): ", z_e)

            # Start projecting y-axis
            y_prime = T0e[0:3, 1]
            print("Intended y: ", y_prime)
            normal_vec_z = z_e
            y_prime_norm = np.linalg.norm(y_prime)
            print("normal_vec_z")
            print(normal_vec_z)
            print("y_prime")
            print(y_prime)
            print("dot prod")
            print(np.dot(y_prime, normal_vec_z))
            y_e = copy.deepcopy(y_prime )
            dot_value = np.dot(y_prime, normal_vec_z)
            normal_dot = [y_prime[i] * dot_value for i in range(len(normal_vec_z))]
            y_e -= normal_dot
            y_e_norm = np.linalg.norm(y_e)
            y_e /= y_e_norm

            print("Feasible y: ", y_e)

            # Start projecting x-axis
            x_prime = T0e[0:3, 0]
            print("Intended x: ", x_prime)

            x_e = np.cross(y_e, z_e)
            x_norm = np.linalg.norm(x_e)
            x_e /= x_norm

            print("Feasible x: ", x_e)

            T0e_feasible[0:3, 0]  = x_e
            T0e_feasible[0:3, 1]  = y_e
            T0e_feasible[0:3, 2]  = z_e

            print("Feasible T0e: ", T0e_feasible)

            # Recursive call to the inverse func
            return inverse(T0e_feasible)

        # Account for joint limits
        exceeds_limits = [False for row in q]

        for row_idx in range(q.shape[0]):
            for theta_idx in range(q.shape[1]):
                if (q[row_idx, theta_idx] > upperLim[0, theta_idx]) or (q[row_idx, theta_idx] < lowerLim[0, theta_idx]):
                    exceeds_limits[row_idx] = (exceeds_limits[row_idx] | True)

        print("Joint limits exceeded: ", exceeds_limits)
        q = q[np.invert(exceeds_limits),:]
        print("q after filtering: ", q)

        # Your code ends here

        return q, isPos

def boundaryCollision(point, boundary):
    outOfBounds = False
    # print("================ Boundary Collision ================")
    # print("Point " + str(point))

    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(point[0])

    for i in range(len(startPos)):

        # Check if joint i is within the boundary
        outOfBounds |= (startPos[i][0] < boundary[0])
        outOfBounds |= (startPos[i][1] < boundary[1])
        outOfBounds |= (startPos[i][2] < boundary[2])
        outOfBounds |= (startPos[i][0] > boundary[3])
        outOfBounds |= (startPos[i][1] > boundary[4])
        outOfBounds |= (startPos[i][2] > boundary[5])

    return outOfBounds

def obstacleCollision(start, goal, obstacles):
    """
    start is a pose containing q1 -> qe
    goals is a pose containing q1 -> qe
    returns if there is a feasible straight line path
    """
    lineCollision = False
    # print("================ Obstacle Collision ================")
    # print("lineCollision is " + str(lineCollision))
    # print("Start " + str(start))
    # print("Goal " + str(goal))

    f = calculateFK()

    # All the joint XYZ values
    # Use index 0 for start and goal because they are
    # nested lists
    startPos, _ = f.forward(start[0])
    goalPos, _ = f.forward(goal[0])

    # print("StartPos " + str(startPos))
    # print("GoalPos " + str(goalPos))

    for obstacle in obstacles:
        # Iterate through every joint
        for i in range(len(startPos)):
            results = detectCollision([startPos[i]], [goalPos[i]], obstacle)
            for result in results:
                lineCollision |= result

    # print("lineCollision is " + str(lineCollision))
    return lineCollision

def findNN(points, newPoint):
    """
    Return i such that points[i] is the closest distance
    from newPoint for all the points in the array
    """

    minI = -1
    minDist = 99999
    for i in range(len(points)):
        # find distance
        a = points[i]
        b = newPoint
        dist = np.abs(sum([(a[i] - b[i])**2 for i in range(3)])**0.5)

        # update minI
        if dist < minDist:
            minI = i
            minDist = dist

    return minI

def graphTrajectory(points):

    # Data for a three-dimensional line
    zline = []
    xline = []
    yline = []
    f = calculateFK()
    for q_ix in range(0, len(points), 10):
        endXYZ = f.forward(points[q_ix])[0][-1]
        zline.append(endXYZ[2])
        yline.append(endXYZ[1])
        xline.append(endXYZ[0])


    # ax = plt.axes(projection='3d')
    #
    # ax.plot3D(xline, yline, zline, 'gray')
    # plt.show()

def deleteElement(array, left, right) :
    j = 0
    for i in range(len(array)) :
        if i <= left or i >= right :
            array[j] = array[i]
            j += 1

def postProcessing(points, obstacles):
    processed = deepcopy(points)
    i = 0
    maxIter = len(points)
    while(i < maxIter):

        a = random.randrange(0, len(points))
        b = random.randrange(0, len(points))

        if(a == b): continue
        coll = obstacleCollision([points[a]], [points[b]], obstacles)

        if not coll: deleteElement(processed, a, b)
        i += 1

    print(processed)
    return processed

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (1x6).
    :param goal:        goal pose of the robot (1x6).
    :return:            returns an mx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix..
    """

    #map_struct = loadmap("maps/map5.txt")
    # start = np.array([1.140773925689457, 0.11726018970498742, 1.0621186359361474, 1.56795931069834, -1.9240993391887418, 0.0])
    # goal = np.array([0, 0, 0, 0, 0, 0])
    # path = Astar(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    # print(path)

    #start = np.array([0, 0, 0, 0, 0, 0])

    # goalXYZ = [-275, 0, 375]
    # goalDeg = np.array([0, -15, -111, -11, 0, 0])
    # goalDeg = np.array([0, -3.5, -111, -11, 0, 0])
    # goal = np.radians(goalDeg)

    obstacles = map.obstacles
    base1=np.array([[-30,-30,0.5,-0.1,-0.1,80]])
    base2=np.array([[0.1,0.1,0.1,30,30,80]])
    boundary = map.boundary
    bufferRadius = 40

    #31.75
    # subtract bufferRadius from start XYZ
    # and add to end XYZ
    for obstacle in obstacles:
        obstacle[0] = max(obstacle[0] - bufferRadius, boundary[0])
        obstacle[1] = max(obstacle[1] - bufferRadius, boundary[1])
        obstacle[2] = max(obstacle[2] - bufferRadius, boundary[2])
        obstacle[3] = min(obstacle[3] + bufferRadius, boundary[3])
        obstacle[4] = min(obstacle[4] + bufferRadius, boundary[4])
        obstacle[5] = min(obstacle[5] + bufferRadius, boundary[5])


    # T0e = [ [0, 0, 1, -350],
    #         [0, -1, 0, 0],
    #         [-1, 0, 0, 400],
    #         [0, 0, 0, 1]]
    # print(inverse(np.array(T0e)))
    print(obstacles)

    print(obstacles)
    # print(boundary)

    if (np.array_equal(goal, start)):
        print("start equals goal")
        return [start]

    f = calculateFK()
    startPos, _ = f.forward(start)
    goalPos, _ = f.forward(goal)

    # Desired XYZ of end-effector at goal
    startE = startPos[-1]
    goalE = goalPos[-1]

    # Desired opening width of end-effector at goal
    goalEWidth = goal[-1]

    print("Start XYZ: ", startE)
    print("Goal XYZ: ", goalE)

    print("Start Thetas: ", start)
    print("Goal Thetas: ", goal)

    print("Obstacles: ", obstacles)

    # Check if straight line path exists between start and goal
    lineCollision = obstacleCollision([start], [goal], obstacles)

    # check if start or goal pose is inside a C-space obstacle
    startObstacle = obstacleCollision([start], [start], obstacles)
    goalObstacle = obstacleCollision([goal], [goal], obstacles)
    obstacles=np.append(obstacles,base1,axis=0)
    obstacles=np.append(obstacles,base2,axis=0)
    if not lineCollision:
        print ("no collision on straight line")
        return [start, goal]
    # elif(startObstacle or goalObstacle):
    #     print("Target or Start inside obstacle")
    #     return ([])

    # currentPose
    currentPose = start

    # list of feasible points on the line such that
    # feasible line exists between adjacent points
    # in the list
    points = [list(start)]
    goalFound = False

    # total number of iterations
    maxIter = 1000
    i = 0

    # Lower joint limits in radians (grip in mm
    # (negative closes more firmly))
    lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]

    # Upper joint limits in radians (grip in mm)
    upperLim = [1.4, 1.4, 1.7, 1.7, 1.5, 30]

    while (not goalFound and i < maxIter):
        # sample a pose
        randQ1 = random.uniform(lowerLim[0], upperLim[0])
        randQ2 = random.uniform(lowerLim[1], upperLim[1])
        randQ3 = random.uniform(lowerLim[2], upperLim[2])
        randQ4 = random.uniform(lowerLim[3], upperLim[3])
        randQE = random.uniform(lowerLim[4], upperLim[4])

        newPose = [randQ1, randQ2, randQ3, randQ4, randQE, goalEWidth]
        print(i, newPose)

        coll = obstacleCollision([currentPose],[newPose], obstacles)
        coll |= boundaryCollision([currentPose], boundary)

        if not coll:
            points.append(list(newPose))
            currentPose = newPose

            if (not obstacleCollision([newPose],[goal], obstacles)):
                points.append(list(goal))
                print("Straight Line to goal feasible")
                goalFound = True

        i += 1
        if i==maxIter:
            print("max iterations reached")
    print("Before post-processing: " + str(len(points)))
    processed = postProcessing(points, obstacles)
    graphTrajectory(processed)
    print("After post-processing: " + str(len(processed)))

    for q_ix in range(len(points)):
        for joint in range(6):
            endXYZ = f.forward(points[q_ix])[0][joint]
            print("[%i][%i]"%(q_ix, joint) + str(endXYZ) )
            # print("[x]" + str(q) )

    return points
