## emptyMap.txt
---------------------------
start = [0,  0, 0, 0, 0, 0]
end = [0, 0, 1.1, 0, 0, 0]


## map1.txt
---------------------------
start = [0,  0, 0, 0, 0, 0]
end = [0, 0, 1.4, 0, 0, 0]


### Parameters that worked:
- zeta=10
- eta=10e7
- rho0= 100
- alpha=0.1


## map2.txt
---------------------------
start = [-1.2,  0, 0, 0, 0, 0]
end = [1.4, 0, 0, 0, 0, 0]

### Parameters: Same as map 1

## map3.txt
---------------------------
start = [0,  0, 0, 0, 0, 0]
end = [0, 0, 1.4, 0, 0, 0]

### Parameters that worked:
- zeta=10
- eta=10e5
- rho0=10
- alpha=0.1

---------------------------
start = [0,  0, 0, 0, 0, 0]
end = [1.4,  0, 1, 0, 0, 0]

### Parameters: same as map 3 above

### map4.txt
---------------------------
start = [1, 0.5, 0, 0, 0, 0]
end = [0, -1.2, 0, 0, 0, 0]

### Parameters: Same as map 3

---------------------------
start = [0, 0, 0, 0, 0, 0]
end = [0, 0, -1.4, 0, 0, 0]

### Parameters: same as map 1


### map5.txt
---------------------------
start = [0, 0, 0, 0, 0, 0]
end = [0.5, -0.5, -1.4, -1, 0, 0]

### Parameters: same as map 3


## TODO
[o] turn path cleaning back on
- find general eta that works for every map
[o] turn `rrt_count` checker back up to 99
[o] uncomment return path inside `rrt_count` bound condition
[o] remove `Linear search` section in `potentialFieldPath`

## Tests

## Map 1 tests
- eta searched from 10e5 to 10e7 with 10 points
- rho0 searched from 10 to 100 with 50 points

