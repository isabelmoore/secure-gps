#!/usr/bin/env python3

from audioop import cross
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


# Ensure pathArray2 has the same length as pathArray1
if len(pathArray2) > len(pathArray):
    closest_indices = []
    for point1 in pathArray:
        # Calculate the distance between point1 and all points in pathArray2
        distances = [np.linalg.norm(point1 - point2) for point2 in pathArray2]
        closest_index = np.argmin(distances)
        closest_indices.append(closest_index)

    pathArray2 = pathArray2[closest_indices]
    print(pathArray2)

elif len(pathArray) > len(pathArray2):
    closest_indices = []
    for point2 in pathArray2:
        # Calculate the distance between point2 and all points in pathArray1
        distances = [np.linalg.norm(point2 - point1) for point1 in pathArray]
        closest_index = np.argmin(distances)
        closest_indices.append(closest_index)
    pathArray= pathArray[closest_indices]

# Compute the cross track error for each pair of points
cross_track_errors = [np.linalg.norm(point1 - point2) for point1, point2 in zip(pathArray, pathArray2)]

# Create time points for the x-axis (assuming each point corresponds to a time step)
time_points = np.arange(len(pathArray))

# Plot cross track error as a function of time


plt.figure(figsize=(10, 6))
plt.plot(time_points, cross_track_errors, label='Cross Track Error (m)')
plt.xlabel('Time (s)')
plt.ylabel('Cross Track Error')
plt.title('Cross Track Error vs. Time')
plt.legend()
plt.grid(True)

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


# Show the plot
plt.show()
