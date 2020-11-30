import re, seaborn as sns
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import ListedColormap
import pandas as pd
import seaborn as sns

#PART 3 EVALUATION: CIRCLE TRAJECTORY
# Data for a three-dimensional line
zline = []
xline =[]
yline = []
mf=2
v=np.array([0,1*mf,-1*mf])
omega=np.array([np.NaN,np.NaN,np.NaN])

#for loop through different time steps
q=np.array([0,0,0,0,0,0])
x = 0.01
R = calculateFK()
jointNum = 5
#y=np.sqrt(25-(x**2))
#qdot=IK_velocity (q, v, omega, jointNum)
IV=100
for t in range(0,IV):
    qdot=IK_velocity (q, v, omega, jointNum)
    q=q+qdot
    #v=v+np.array([0,0.01,-0.01])
    v1=(-1-1)/IV*mf
    v2=(-1+1)/IV*mf
    v=v+np.array([0,v1,v2])
    #print(np.round(v[1]**2,6))
    #omega=np.array([np.NaN,0,np.NaN])
    #v,omega=FK_velocity (q, qdot,jointNum)
    Jp,_=R.forward(q)
    xline.append(Jp[5][0])
    yline.append(Jp[5][1])
    zline.append(Jp[5][2])
    #print(Jp[jointNum])
#v=np.array([0,-1,-1])
for t in range(0,IV):
    qdot=IK_velocity (q, v, omega, jointNum)
    q=q+qdot
    v1=(-1+1)/IV*mf
    v2=(1+1)/IV*mf
    v=v+np.array([0,v1,v2])
    Jp,_=R.forward(q)
    xline.append(Jp[5][0])
    yline.append(Jp[5][1])
    zline.append(Jp[5][2])
#v=np.array([0,-1,1])
for t in range(0,IV):
    qdot=IK_velocity (q, v, omega, jointNum)
    q=q+qdot
    v1=(1+1)/IV*mf
    v2=(1-1)/IV*mf
    v=v+np.array([0,v1,v2])
    Jp,_=R.forward(q)
    xline.append(Jp[5][0])
    yline.append(Jp[5][1])
    zline.append(Jp[5][2])
#v=np.array([0,1,1])
for t in range(0,IV):
    qdot=IK_velocity (q, v, omega, jointNum)
    q=q+qdot
    v1=(1-1)/IV*mf
    v2=(-1-1)/IV*mf
    v=v+np.array([0,v1,v2])
    Jp,_=R.forward(q)
    xline.append(Jp[5][0])
    yline.append(Jp[5][1])
    zline.append(Jp[5][2])

ax = plt.axes(projection='3d')
ax.plot3D(xline, yline, zline, 'red')
plt.xlabel("X axis")
plt.ylabel("Y axis")

fig, axs = plt.subplots(2, 2)

axs[0, 0].plot(xline, yline)
axs[0, 0].set_title('Top View', fontsize=10)
axs[0, 0].axis('equal')
axs[0, 0].set_xlabel("X Axis")
axs[0, 0].set_ylabel("Y Axis")


axs[1, 0].plot(xline, zline)
axs[1, 0].set_title('Side View', fontsize=10)
axs[1, 0].axis('equal')
axs[1, 0].set_xlabel("X Axis")
axs[1, 0].set_ylabel("Z Axis")

axs[1, 1].plot(yline, zline)
axs[1, 1].set_title('Front View', fontsize=10)
axs[1, 1].axis('equal')
axs[1, 1].set_xlabel("Y Axis")
axs[1, 1].set_ylabel("Z Axis")

fig.tight_layout()

plt.show()
