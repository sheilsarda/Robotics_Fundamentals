# MEAM520 Individual Report 
## Sheil Sarda - Team 10

### Peer Evaluation

| Team Member | Score |
|-------|------|
| Venkata Gurrala | 2.5/2.5 |
| Brian Grimaldi | 2.5/2.5 |
| Sheil Sarda | 2.5/2.5 |
| Alexis Ward | 2.5/2.5 |

### Most challenging issue and how you dealt with it

#### The most challenging issue we ran into this project was how to pick-up dynamic blocks, and successfully stack them onto our goal platform.

##### Should the end-effector pick up a block from anywhere in the reachable workspace

Until pretty late in our development process, we were still debating whether or not to pick up blocks from anywhere in the reachable workspace of the robot or only from a fixed angle of the turntable. Following our first scrimmage, we took inspiration from other teams to evaluate which approach yields the most benefit and is the most robust. This criteria helped us select the fixed angle approach, since it simplifies the future decision making process of how to select the block to pick-up, how to scoop it up from the turntable, etc. as I have described in the next sections.

Had we gone down the other path of picking up blocks from any approach angle, we would have likely ran into issues around the robustness of the algorithm and had to backtrack to this approach anyway, so this decision saved us a lot of wasted time.

##### How to select which block to pick up
We evaluated several metrics to optimize for, including:

- time till a block became reachable
- closest Cartesian distance to the goal platform
- farthest from the center of the turntable

We decided that since our end-effector is only going to pick up blocks from a certain approach angle to the turntable center, we should select the block which will line up with the end-effector approach angle first, and is simultaneously on the outermost possible radius for that angle.

##### Best scooping motion of the block

We evaluated several approaches here as well:

- Claw-machine motion by picking-up exclusively perpendicular to the turntable
- Scooping motion by thrusting the block into the end-effector parallel to the turntable

We decided to go with the scooping motion because empirically it led to better performance than the claw-machine motion of lining up the end-effector exactly on top of the robot. In hindsight, this superior performance is caused by the fact that scooping up a block parallel to the turntable is more tolerant of errors related to localizing the block since the block is more likely to correctly re-orient itself based on the end-effector gripper as it gets thrust in.

In contrast, if we were to pick up the block vertically, we could not pick up blocks if the orientation of the end-effector does not exactly line up with that of the block directly beneath it, or if it doesn't get there at just the right time before the block goes past.

##### Quickest path back to goal platform

While we were confident that our RRT and potential field planning methods would successfully get us to the goal platform, since we had a fixed platform wait and pick-up position, and we knew where we wanted to stack the dynamic blocks, we were able to eliminate the latency of running the path planning algorithm by hard-coding in fixed way points to get us to and from the turntable to the goal platform without knocking off any blocks.

This process was very time-consuming, since we had to manually try out a lot of robot poses, and make fine-grain adjustments to ensure the block was release on top of the existing stack, and that no collisions occurred during the motion of the arm.

##### Max Stacking height

Empirically determine the highest feasible stack length was another time-consuming exercise in finding a compromise between the structural integrity of the tower and its height. Through our experiments we determined 3 was the optimal height to ensure sufficient robustness during the competition, so we hard-coded  in this limit.

##### How to handle corner cases (Infinite wait, Unreachable blocks, etc.)

During our competition last week, we encountered an opponent twice whose strategy for handling dynamic blocks was to use the robot arm to sweep off as many blocks from the turntable before proceeding to stack their static blocks. During our first bout with our robot, this was catastrophic for our robot because our algorithm failed to realize that the block it was waiting to pick up had been swept off the platform seconds ago.

However, we successfully overcame this obstacle before our second bout with the same robot by dynamically refreshing the list of blocks on the turntable, and aborting our pick-up if the block has disappeared.

Another corner case we have considered is how to handle blocks which are located at a far enough radius away from the center of the turntable to collide with our robot end-effector as it waits to pick up an inner dynamic block. What currently occurs in this case is that the block is effectively stopped by the mass of the wrist of our robot until the block our robot is waiting for comes around and gets picked up.

As a potential next step, we can implement a collision detection algorithm which tracks when a dynamic block has collided with the wrist of the end-effector and pulls up the end-effector for a brief period to let the block pass, similar to how a drawbridge dam lifts up to let ships pass underneath. Implementing this will avoid undefined behavior of the block sometimes squeezing shut the end-effector because of the horizontal force it is exerting. 

### How this project demonstrates technical growth on your part since the beginning of the semester

This was an awesome learning opportunity to put together a lot of the tools and ideas we developed throughout the semester, and also to benchmark ourself against our peers in an apples-to-apples way.


Thank you very much Prof. Sung and all the wonderful TAs for organizing this assignment.
