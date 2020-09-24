import matplotlib.pyplot as plt
import numpy as np

x = []
y = []
z = []

for t2 in np.arange(-1.2, 1.4, 0.1):
    for t3 in np.arange(-1.8, 1.7, 0.1):
        for t4 in np.arange(-1.5, 1.7, 0.1): 
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

plt.plot(x, y, 'bo')
plt.savefig('x_y_plot.png')

plt.plot(y, z, 'bo')
plt.savefig('y_z_plot.png')

plt.plot(x, z, 'bo')
plt.savefig('x_z_plot.png')
