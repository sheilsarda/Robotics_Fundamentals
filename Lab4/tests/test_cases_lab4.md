# For the FK we did every possible combination of each joint, each q from the following list, and each dq from the following list.

# From there we compared the accuracy of the linear and angular velocity to the solution.

```
q_list = [[0,0,0,0,0,0];
           [0.6463,    0.7094,    0.7547,    0.2760,    0.6797,    0.2];
           [0.1626,    0.1190,   0.4984,    0.9597,    0.3404,    0.5853];
           [0.2238,    0.7513,    0.2551,    0.5060,    0.6991,    0.8909];
           [0.9593,    0.5472,    0.1386,    0.1493,    0.2575,    0.8407]];

dq_list =  [[0, 0, 0, 0, 0, 0];
            [1, 0, 0, 0, 0, 0];
            [0, 1, 0, 0, 0, 0];
            [0, 0, 1, 0, 0, 0];
            [0, 0, 0, 1, 0, 0];
            [0, 0, 0, 0, 1, 0];
            [0, 0, 0, 0, 0, 1];
            [1, 1, 1, 1, 1, 1]];
```

# The IK tests were more complicated.  For every combination of the  joints, each q in the above list, and each dq in the following list

```
dq_list =  [[0.2, 0.3, -3, 2, 0.1, 0];
            [.14, -2, 1.7, 1.7, 0, 0];
            [0, 1.2, 1.2, 15, 0, 0];
            [0, -1, 0, -2, -3, 0];
            [1, 1, 1, 1, 1, 1]];
```

# We ran the following test:

# Let [v,w] = FK_velocitySol(q, dq, joint). We then tested the IK code with the following target  linear and angular velocities:

```
[v,w]
[v.*v,w]
[v.*w, v.*w]
[v, w.*w]
[v,[nan,nan,nan]]
[[nan,nan,nan],[nan,nan,nan]]
```

# We then plugged the resulting dq into the solution FK velocity code before comparing the tests to the solution.
