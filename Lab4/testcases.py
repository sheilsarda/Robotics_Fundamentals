from FK_velocity import FK_velocity





def main():

  q_list = [[0,0,0,0,0,0],
             [0.6463,    0.7094,    0.7547,    0.2760,    0.6797,    0.2],
             [0.1626,    0.1190,   0.4984,    0.9597,    0.3404,    0.5853],
             [0.2238,    0.7513,    0.2551,    0.5060,    0.6991,    0.8909],
             [0.9593,    0.5472,    0.1386,    0.1493,    0.2575,    0.8407]]

  dq_list =  [[0, 0, 0, 0, 0, 0],
              [1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1]]

  a = FK_velocity (q[0], dq[0], 1)
  print(a)
  