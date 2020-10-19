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
        
    
