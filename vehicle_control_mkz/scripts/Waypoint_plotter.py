#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np 
import os 

here = os.path.dirname(os.path.abspath(__file__))

filename = os.path.join(here, 'odom_waypoints.dat')
pathArray = []

with open(filename,"r") as f: 
    for line in f:
        pathArray.append(line.strip())
    pathArray = np.array([list(map(float, x.split(','))) for x in pathArray])
    pathArray = np.array([[float(y) for y in x] for x in pathArray])




fig, ax = plt.subplots()

# plot the pathArray using a blue line with circle markers
ax.plot(pathArray[:,0], pathArray[:,1], '-o', color='blue')

# set the x and y axis labels
ax.set_xlabel('X-coordinate')
ax.set_ylabel('Y-coordinate')

# set the title of the plot
ax.set_title('Waypoints')

# show the plot
plt.show()
