import re, seaborn as sns
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import ListedColormap
import pandas as pd
import seaborn as sns

#PART 3 EVALUATION: LINEAR TRAJECTORY
# Data for a three-dimensional line
zlinel = []
xlinel =[]
ylinel = []
v=np.array([-1,0,0])
omega=np.array([0]*3)

#for loop through different time steps
q=np.zeros((6))
x = 0.01
R = calculateFK()
jointNum = 5
qdot=IK_velocity (q, v, omega, jointNum)
for t in range(0,100):
    q=q+qdot
    qdot=IK_velocity (q, v, omega, jointNum)
    Jp,_=R.forward(q)
    xlinel.append(Jp[jointNum][0])
    ylinel.append(Jp[jointNum][1])
    zlinel.append(Jp[jointNum][2])

ax = plt.axes(projection='3d')
ax.plot3D(xlinel, ylinel, zlinel, 'red')
plt.xlabel("X axis")
plt.ylabel("Y axis")

fig, axs = plt.subplots(2, 2)

axs[0, 0].plot(xlinel, ylinel)
axs[0, 0].set_title('Top View', fontsize=10)
axs[0, 0].axis('equal')
axs[0, 0].set_xlabel("X Axis")
axs[0, 0].set_ylabel("Y Axis")


axs[1, 0].plot(xlinel, zlinel)
axs[1, 0].set_title('Side View', fontsize=10)
axs[1, 0].axis('equal')
axs[1, 0].set_xlabel("X Axis")
axs[1, 0].set_ylabel("Z Axis")

axs[1, 1].plot(ylinel, zlinel)
axs[1, 1].set_title('Front View', fontsize=10)
axs[1, 1].axis('equal')
axs[1, 1].set_xlabel("Y Axis")
axs[1, 1].set_ylabel("Z Axis")

fig.tight_layout()

plt.show()
