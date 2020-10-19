## Pseudocode for simple RRT

1. if goal point is equal to start point (we reached it)
1. if line segment from qstart to end doesnt collide (we reached it)
1. if it does collide, set current position as start position
    1. while current position not equal to qgoal and we are less than max iterations
        1. choose random point in space and make a line segment from current  position to that point
        1. if new line segment collides, break out and start for loop again
        1. if it reaches goal position (we reached it)
        1. else store the new coordinate in an array and run loop again with this coordinate as current position
1. Find the distance of all the line segments added up and divide by 6 (this is the distance increments used to mark positions)
1. from start point, measure out the first distance increment with line segments
1. mark where the position is and use FK to find joint angles for position only (not orientation)
1. input orientation at the last point only

## TODO
- Check feasibility of orientation of end effector
    OR 
  Assume start and end orientations are feasible
- What are the robot joint positions while the end effector traverses along the path

    - Knowns:
        - X, Y and Z end-effector waypoints between start and end by following RRT approach
            detect feasibility for joints

            given end effector xyz + Orientation, we know the positions of the 
        
        
        - q0-q5 waypoints between start and end which might not be feasible
            - Joint limits
            - Obstacle detection
        
    - Pick orientations based on goal orientation 
      OR 
      Pick orientation based on obstacle avoidance

## Pseudocode for turning waypoints into `q` array

1. For waypoints, form T0e using XYZ from our while loop and match wrist orientation of the initial point
1. Solve for thetas using IK
1. If no feasible orientation, then find nearest feasible orientation
1. Get joint positions for each of the other joints
1. Model each link as a line between XYZ positions of the joints
1. Detect if the link / line collides with any obstacle
1. If collision, generate a new path from the nearest feasible point
OR
generate new path from start position
1. If no collision on any waypoint in the path, then the robot body does not collide


