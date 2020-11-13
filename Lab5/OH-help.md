# Pre-Lab Questions

## Write a mathematical description of a potential ﬁeld controller for the Lynx robot (i.e., what does your controller look like in equation form?) Explain your choices

- Force of attraction and repulsion we would use, and the parameters used to specify those.

- Controller should be based on one of those equations.
    There should be a system of 2 or several equations that define the controller.
- `B = M * N`
    `C = X / Y`
    `A = B + C`
    Those are the equations
- What formulas we used to define those and the torque calculation.

## The performance of your potential ﬁeld will change depending on what you choose for your speciﬁc parameters. For 1 parameter in your controller, predict what aﬀect increasing/decreasing this parameter will have on your planner performance. Explain

- Zeta (Attraction - field strength of attraction force)
    N (Repulsion)
    Other parameters: Origin of each joint, etc.
- Does she want only one parameter or both?
    Based on what I understand, you can choose just one parameter.
    Without any risk, do it for both parameters.
- Planner performance close and far away from the obstacles. What is your expectation for the Lynx robot?
    We want the trajectory that takes less time and shorter movement.
- How will the eta change the torque?
- Transpose of Jacobian * net force = torque
    How does it influence q (joint movement in c-space)
- How does the repulsive force affect next joint q_i
    -> net force = attractive - repulsive force (zeta and eta)
- If you reduce alpha_i, you will move less in one step. Larger alpha makes you move more. These parameters don't impact the torque equation.
- We set up threshold. How does it impact the trajectory?
- If alpha * normalized force is larger, each pair of waypoints has a larger distance
- This is under the condition that distance between current and desired position is smaller than threshold. However, if the distance is larger than threshold, what happens to position?
- This is a way to avoid attraction to local minima. Determined empirically.

## Design and describe a (set of) evaluation tests to check whether your prediction is correct. What environment will you use. What will you measure

- Are you expecting to see a certain number of tests.
Explain as much as possible. Why 3 tests for e.g.?
- 3 variables, so we are going to test this variable first. Hold the first one constant, change the other one.
- Design and describe-- pretty open-ended.
    We are going to use the Lynx robot environment...
    ROS environment...
    etc.

## Pseudocode on procedure page (Slide 20)

- `q` is the goal point
- Is there a case where end-effector ever gets stuck or passes if statement #2?
    it gets stuck in the local minima and does not move anywhere.

## Do we get repulsive and attractive force for every joint? (Slide 20)

- Yes, you need to care about it for every joint
- the final torque in the pseudocode is the sum of the torque for every joint?
- After you compute attractive and repulsive force for each joint, you should have a Tao for each joint
- For pseudocode, the Tao holds in general but it should be a vector with 6 Taos for each q
- Every Tao is a vector with magnitude and direction, so we cannot directly add them together
- Our robot's Tao should have 6 rows

## Dimension of each of the parameters

- Tao
- `q` is 1 x 6 where each element is the joint angle in the configuration
- For one point and one joint, you should have a unique attractive and repulsive force
- Use the forces to compute separate attractive and repulsive Tao
- If they are both acting on one point, then you have a combined Tao
- For the next joint, you cannot add the old joint's Tao to the next joint's Tao because different positions.

## Difference between parabolic and conic well potential (slide 12 and 13)

- Why do we prefer parabolic well potential?
    -> If you use conic well, it means that if you are trying to get to the goal there will be a high amount of force.
- Imagine if you put a marble down a conical surface, it oscillates a lot closer to the hole. Applying that analogy to robots, the trajectory would not converge and keep on overshooting, causing oscillations.
- The parabolic is used to move slowly to the goal point.
- Huge vibration at the goal position. This is the discontinuity. This discontinuity can cause instability.

## Why are columns for the joints after origin zeroes

- No idea. Go to her OH to ask
