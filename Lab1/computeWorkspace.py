import matplotlib.pyplot as plt
import numpy as np
import pandas


x = []
y = []
z = []

for t2 in np.arange(-1.2, 1.4, 0.05):
    for t3 in np.arange(-1.8, 1.7, 0.05):
        for t4 in np.arange(-1.5, 1.7, 0.05): 
            for t1 in np.arange(-1.4, 1.4, 0.05):
                x.append(
                  np.cos(t1)*(
                    197.325*np.cos(t3+t2)+
                    146.05*np.sin(t2)+
                    68*np.cos(t4+t3+t2))
                )
                y.append(
                  np.sin(t1)*
                  (187.325*np.cos(t3+t2)+
                    146.05*np.sin(t2)+
                    68*np.cos(t4+t3+t2))
                )
                z.append(
                  146.05*np.cos(t2)+
                  76.2-
                  187*np.sin(t3+t2)+
                  68*np.sin(t4+t3+t2)
                )

df = pandas.DataFrame(data={"x": x, "y": y, "z": z})
df.to_csv("./sim_data.csv", sep=',',index=False)

x1 = []
y1 = []

for t2 in np.arange(-1.2, 1.4, 0.01):
    for t1 in np.arange(-1.4, 1.4, 0.01):
                x1.append(
                  np.cos(t1)*(
                    197.325*np.cos((-np.pi/2)+t2)+
                    146.05*np.sin(t2)+
                    68*np.cos(0+(-np.pi/2)+t2))
                )
                y1.append(
                  np.sin(t1)*
                  (187.325*np.cos((-np.pi/2)+t2)+
                    146.05*np.sin(t2)+
                    68*np.cos(0+(-np.pi/2)+t2))
                )

plt.plot(x1, y1, 'bo')
fig.set_size_inches(18, 18)
plt.savefig('x_y_plot.png')
plt.clf()

plt.plot(y, z, 'go')
fig.set_size_inches(18, 18)
plt.savefig('y_z_plot.png')
plt.clf()

plt.plot(x, z, 'ro')
fig.set_size_inches(18, 18)
plt.savefig('x_z_plot.png')
plt.clf()

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)
fig.set_size_inches(18, 18)
plt.savefig('x_y_z_plot.png')

plt.show()