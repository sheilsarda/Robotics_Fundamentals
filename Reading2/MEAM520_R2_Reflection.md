# Grasping POMDPs

## Summary

### Description

- A POMDP model consists of
  1. finite set of states S
  1. actions A
  1. observations O
  1. reward function R that maps each underlying state-action pair into an immediate reward
  1. state-transition model P that specifies a probability distribution over the resulting states 
  1. observation model that specifies the probability of making an observation o in a state s
- In a POMDP, an action has to be specified for every probability distribution over states in the space; the policy will know what to do when the robot is completely uncertain about its state, or when it has competing possibilities

### Main new thing discovered

- By modeling the initial uncertainty using a probability distribution, rather than a set, and doing the same for uncertainties in dynamics and sensing, we are in a position to make trade-offs when it is not possible to succeed in every possible situation
  - We can choose plans that optimize a variety of different objective functions involving those probabilities, including the plan most likely to achieve the goal
  - The probabilistic representation also affords an opportunity for enormous computational savings through a focus on the parts of the space that are most likely to be encountered
  - We can model the problem of choosing actions under uncertainty as a partially observable Markov decision process (POMDP)
- The action set includes guarded motions through free space, as well as compliant motions, in which the robot is constrained to maintain an existing contact while moving to acquire another one
  - Compliant actions serve as funnels, producing configurations with multiple contacts between the robot and an object and generating information about the underlying state
  - A guarded motion causes the robot to move along some vector until it makes or breaks a contact or reaches the limit of the workspace

### Connection to topics in class

## Challenges

### Major challenges

### How the authors addressed this challenge

## Evaluation

- The authors assume that a reasonably accurate model of the task dynamics and sensors is known, and that the principal uncertainty is in the configuration of the robot and the state of the objects in the world
- Consider a 2D cartesian robot with a finger, and a block on the table. The robot has contact sensors on the tip and each side of the finger
- The robot's goal is to have the tip of its finger on top of the block

### Methods

The authors

### Strengths

### Weaknesses

### Score the paper according to our lab report rubric. Review the document and provide justification

| Category      | Score |
|------------   |-------|
| Completeness  | 4     |
| Method        | 5     |
| Evaluation    | 4     |
| Analysis      | 5     |
| Clarity       | 5     |

## List any questions you have about the paper
