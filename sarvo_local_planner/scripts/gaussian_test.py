#!/usr/bin/env python

from matplotlib import transforms
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator

# define normalized 2D gaussian
def gaus2d(x=0, y=0, mx=0, my=0, sx=1, sy=1, A=1):
    return A * 1. / (2. * np.pi * sx * sy) * np.exp(-((x - mx)**2. / (2. * sx**2.) + (y - my)**2. / (2. * sy**2.)))


def circular(x=0, y=0, thr=1):
    z = np.zeros_like(x)
    for i in range(0, len(z)):
        z[i] = thr - np.hypot(x[i], y[i])
    
    for i in range(0, len(z)):
        for j in range(0, len(z)):
            if z[i][j] < 0:
                z[i][j] = 0
    return z


fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

x = np.linspace(-5, 5)
y = np.linspace(-5, 5)

rho_x = 1.2
rho_y = rho_x/2.0
A = 1 * (2. * np.pi * rho_x * rho_y)
x, y = np.meshgrid(x, y) 

z = gaus2d(x, y, 0, 0, rho_x, rho_y, A)
# z = circular(x, y, 1.75)

surf = ax.plot_surface(x, y, z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
# Customize the z axis.
ax.set_zlim(0.0, 1.0)
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()

print("End")