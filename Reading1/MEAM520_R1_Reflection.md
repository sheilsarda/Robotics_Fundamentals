# A Comparison of Transforms and Quaternions in Robotics

## Summary: Write a 3-5 sentence high-level description of the paper contents. What was the main new thing discovered? What is the connection to the other topics we cover in class

### Description

### Main new thing discovered

The main focus of this paper is an alternate approach to modeling rotations and translation using quaternion / vector pairs as spatial operators.

- Quaternions subsume all the properties of real and complex numbers with the exception of commutativity of multiplication.
- For our purposes, it will be convenient to treat a quaternion as a sum of a scalar `s` and a 3-D vector `<<x, y, z>>`. Thus, vectors can be represented as quaternions with null scalar parts.

### Connection to topics in class

We studied the use of vectors to express spatial relationships, and this is an alternative to that approach. Quaternions are superior in the following ways:

- Vector calculus does not lend itself naturally to representing 3D rotations.
- Only unit quaternions are used as rotation operators - a quaternion `q` with `N(q)` not equal to 1 will produce a vector which is oriented correctly, but whose magnitude does not equal that of the original vector.
- Since speed of arithmetic operations in conventional computers tends to exceed that of operand fetching (main memory references), quaternions have an advantage over homogenous transforms as fewer values need to be brought from main memory.
- Total cost of composing quaternion spatial transformations is 31 multiplies and 27 adds.
- In computational expense, the homogenous transform inverse requires a total of 15 multiplications and 9 additions
  - One cross product to recover $\bar{n}$
  - Three dot products to compute new value of $\bar{p}$
  - Pointer rearrangements to accommodate the transpose operation
- Normalization procedure cost
  - For rotational matrices is very sequential-- requiring 6 processors and 12 CPU cycles to carry out the computations
  - For quaternion operators-- requires 4 processors and 5 CPU cycles

## Evaluation: Score the paper according to our lab report rubric. Review the document and provide justification

| Category 	    | Score |
|------------   |---	|
| Completeness 	| 5  	|
| Method 	    | 3 	|
| Evaluation 	| 4  	|
| Analysis  	| 5  	|
| Clarity  	    | 3 	|

## Strengths: What did the paper do well? Were the methods sufficiently explained? Did the authors evaluate the work and provide a complete analysis? Was the writing and organization clear and understandable

## Weaknesses: What could be improved to make the paper stronger

## List any questions or additional comments you have about the paper
