## Pseudocode for `rrt(map, start, goal)`

1. if goal pose is equal to start pose, OR if no obstacles between `start` and `goal` 
    1. return `[start, goal]` 
1. Set current position as `start` position
    1. While current pose not equal to `goal` and loop counter is less than max iterations
        1. Generate random pose using joint limits as the allowed range
        1. Check if line segment from current pose to randomly generated pose collides
            1. If collision detected, break out and start for loop again
        1. If the newly generated point has a collision-free path to the goal pose, append both the random pose and goal pose to the path 
        1. Else, store the random pose in the path array, set the current pose to the random pose, and continue to the next iteration of the while loop.
1. Post-process the path to prune the generated RRT.
1. Return the path.

## Pseudocode for planning in C space

1. Why plan in configuration space
1. How does collision detection work in configuration space
    1. Given q0-q5 (randomly sampled)
    1. Detect if valid configuration (within joint limits)
    1. Plug into FK to get XYZ
    1. Send to detectObstacle function


## Pseudocode for isCollided

1. Take in q values to compute joint positions
1. use collision function to detect if any joint is inside an obstacle


## Approximate volume of robot

1. Put robot in Zero configuration in Gazebo
1. Approximate volume using cylinders


