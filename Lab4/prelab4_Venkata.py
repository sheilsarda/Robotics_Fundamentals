import numpy as np
from calculateFK import calculateFK

# inputs
q=[0,np.pi/4,0,0,0,0]
qdot=[0,2,0,0,0,0]
z0=np.array([0,0,1])

# z matrix calculated for joints 1,2 and 3 at once
z13=np.array([-np.sin(q[0]),np.cos(q[0]),0])

# taking an instance of the calculate FK function
fk = calculateFK()
Jp,T0e = fk.forward(q)

# using the 3rd column of the T0e matrix to find z matrix for joints 4 and e
z4e = np.array([T0e[0,2],T0e[1,2],T0e[2,2]])

# for loop to fill complete jacobian matrix
J = np.zeros((3,6))
for i in range(1,6):
    # Finding the difference between joint origin and end effector  
    Jo=Jp[5]-Jp[i-1]
    #determining the cross product based on the joint considered
    if i == 1:
        Jr=np.cross(z0,Jo)
    elif i <= 3:
        Jr=np.cross(z13,Jo)
    else:
        Jr=np.cross(z4e,Jo)
        
    # jacobian matrix being filled
    J[0, i-1]=Jr[0]
    J[1, i-1]=Jr[1]
    J[2, i-1]=Jr[2]

print("Jacobian")
print(J)

# multiply result with qdot to get final linear velocity matrix
linv=np.matmul(J,qdot).T
print(linv)