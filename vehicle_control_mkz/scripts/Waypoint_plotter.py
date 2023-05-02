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

filename_r = os.path.join(here, 'follow_waypoints.dat')
pathArray2 = []

with open(filename_r,"r") as f: 
    for line in f:
        pathArray2.append(line.strip())
    pathArray2 = np.array([list(map(float, x.split(','))) for x in pathArray2])
    pathArray2 = np.array([[float(y) for y in x] for x in pathArray2])


fig, ax = plt.subplots()

# plot the pathArray using a blue line with circle markers
ax.plot(pathArray[:,0], pathArray[:,1], '-o', color='blue',label='GPS Position')
ax.plot(pathArray2[:,0], pathArray2[:,1], '-o', color='red',label='MKZ_Position')
# set the x and y axis labels
ax.set_xlabel('X-coordinate')
ax.set_ylabel('Y-coordinate')
legend_drawn_flag = True
plt.legend(["MKZ_Position","GPS Position"], loc=0, frameon=legend_drawn_flag)
# set the title of the plotax.plot(pathArray[:,0], pathArray[:,1], '-o', color='blue')
ax.set_title('Waypoints')

# show the plot
plt.show()
