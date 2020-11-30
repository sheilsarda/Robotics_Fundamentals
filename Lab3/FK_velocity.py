import numpy as np
def FK_velocity (q, dq, joint):
    v = np.array([0, 0, 0])
    omega = np.array([0, 0, 0])

    if(joint == 0 or joint >= 6):
        return([0.0, 0.0, 0.0],[0.0, 0.0, 0.0])
    #FK_Velocity
    Jac=calcJacobian(q,joint)
    FKvel=np.matmul(Jac,dq).T
    v=FKvel[0:3]
    omega=FKvel[3:6]

    print(v, omega)

    return v, omega
