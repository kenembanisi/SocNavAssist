#!/usr/bin/env python

from matplotlib import transforms
import numpy as np
import matplotlib.pyplot as plt

plt.figure()

# get data
samples = [
[ 0.85, -3.2 ] , 
[ 0.85, -1.2 ] , 
[ 0.85, 0.8 ] , 
[ 0.85, 2.8 ] , 
[ 0.85, 4.8 ] , 
[ 2.85, -3.2 ] , 
[ 2.85, 0.8 ] , 
[ 2.85, 2.8 ] , 
[ 2.85, 4.8 ] , 
[ 4.85, -3.2 ] , 
[ 4.85, 0.8 ] , 
[ 4.85, 2.8 ] , 
[ 6.85, -3.2 ] , 
[ 6.85, -1.2 ] , 
[ 6.85, 0.8 ] , 
[ 8.85, -3.2 ] , 
[ 8.85, 0.8 ] , 
[ 8.85, 2.8 ] , 
[ 8.85, 4.8 ] , 
[ 10.85, -3.2 ] , 
[ 10.85, 0.8 ] , 
[ 10.85, 4.8 ] , 
[ 12.85, -3.2 ] , 
[ 12.85, -1.2 ] , 
[ 12.85, 0.8 ] , 
[ 12.85, 2.8 ] , 
[ 12.85, 4.8 ]
]

samples_x = [samples[i][0] for i in range(len(samples))]
samples_y = [samples[i][1] for i in range(len(samples))]

samples_grid_x_ = [-0.15 + 1 + 2*i for i in range(int(14.7/2))]
samples_grid_y_ = [-4.2 + 1 + 2*i for i in range(int(10.0/2))]

samples_grid_x = []
samples_grid_y = []

for x in samples_grid_x_:
    for y in samples_grid_y_:
        samples_grid_x.append(x)
        samples_grid_y.append(y)

# call plotter
plt.plot(samples_x, samples_y, 'bo')
# plt.plot(samples_grid_x, samples_grid_y, 'go')

for i in range(len(samples_x)):
    plt.annotate(str(i), (samples_x[i], samples_y[i]))

plt.xlim(-0.15, 14.60)
plt.ylim(-4.20, 5.86)

plt.grid()

plt.show()
