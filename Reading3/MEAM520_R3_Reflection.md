# Coordinated multi-arm motion planning: Reaching for moving objects in the face of uncertainty

## Summary

### Description

- Using multiple robots extends the reachable workspace and potentially increases the payload of the overall robotic system. 2 instances of this task, where object's motion is non-deterministic:
  - Co-working scenario in which a human hands over a large object to a robot
  - Intercepting a large flying object
- Determining the pick-up point is done by taking into account the workspace of the multi-arm system, recomputed to adapt to changing trajectory of the object
- Task constraints impose position and velocity constraints at the object's interception
- Coordination constraints impose that the robots move in coordination with each other, ensuring that end-effectors do not collide with each other
- Early approaches addressed the coordination problem with master-slave strategies, but when dealing with moving objects, a difficulty arises when the master and slave roles have to be assigned to the arms and communication delays arise

### Main new thing discovered

In order to achieve stable and coordinated reaching motions for multiple arms when the target object is in motion, several problems need to be solved:
  
1. Prediction of the object's trajectory
1. Computing intercept points for each arm
1. Planning coordination motion of the robot arms towards their corresponding intercept points

### Connection to topics in class

- There is an RRT-based algorithm to generate collision free motions to grasp an object at rest with a bi-manual platform using a search-based strategy, however due to its computational complexity it becomes inadequate when trying to reach for a moving object

## Challenges

### Describe one major challenge of coordinated multi-arm planning compared to the single-arm planning we did in class

- Reaching for an object in a smooth and efficient manner requires each hand individually adjusting to the orientation, shape and size of the object
- Handling multiple constraints simultaneously cannot happen within a few milliseconds using traditional optimal control approaches

### Explain how the authors addressed this challenge

- The authors propose an approach which generates coordinated trajectories for a multi-arm robot system that ensures arms will reach the moving object simultaneously
- The authors use autonomous dynamical systems to instantaneously re-plan the coordination motion for each hand-arm robot
- Use a simple ballistic motion algorithm to predict the motion of the real object and propose a controller

## Evaluation

### Methods

- A scenario in which an object with arbitrary shape and mass is moving toward a multi-arm system where the dynamics of the object are unknown since the human carrying the object is blindfolded
- A multi-arm reach is successful if all robot arms simultaneously intercept the object on its feasible reaching points
- A fast moving object is thrown to the robots from 2.5m away, resulting in approximately 0.6 seconds of flying time. Due to inaccurate prediction of the object trajectory, the feasible intercept points need to be updated and refined during the reach

### Strengths: When does the planner work well

- Success rates of experiments are measured by a Boolean metric: success or failure with a 2cm error tolerance. The success rate is 85% with the box object

### Weaknesses: What are the major limitations?

- There is no analytical proof that the system is fast enough to converge on the optimal solution in time
- The authors only control the motion of the arm from initial condition to the point when the arms reach the object and the fingers are about to close on the object; no interaction forces are accounted for
- The algorithm only guarantees collision-free movement of the end-effectors of both robot arms, but does not ensure that the rest of the arms will not collide with each other

### Score the paper according to our lab report rubric. Review the document and provide justification

| Category      | Score |
|------------   |-------|
| Completeness  | 4     |
| Method        | 5     |
| Evaluation    | 4     |
| Analysis      | 5     |
| Clarity       | 5     |

## List any questions you have about the paper
