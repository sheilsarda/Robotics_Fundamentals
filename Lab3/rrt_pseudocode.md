pseudocode starts
if goal point is equal to start point (we reached it)
if line segment from qstart to end doesnt collide (we reached it)
if it does collide, set current position as start position
    while current position not equal to qgoal and we are less than max iterations
        choose random point in space and make a line segment from current position to that point
        if new line segment collides, break out and start for loop again
        if it reaches goal position (we reached it)
        else
            store the new coordinate in an array and run loop again with this coordinate as current position


    Assuming we reach the position by the end of the while loop

Find the distance of all the line segments added up and divide by 6 (this is the distance increments used to mark positions)
from start point, measure out the first distance increment with line segments
mark where the position is and use FK to find joint angles for position only (not orientation)
input orietntaion at the last point only
