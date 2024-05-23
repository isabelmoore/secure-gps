import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns


df1_circle = pd.read_csv('Track_Sensor_CI_Circle.txt')
df1_Lanechange = pd.read_csv('Track_Sensor_CI_LaneChange.txt')
df1_Straight= pd.read_csv('Track_Sensor_CI_Straight.txt')

df2_circle = pd.read_csv('Track_Sensor_KF_Circle.txt')
df2_Lanechange = pd.read_csv('Track_Sensor_KF_LaneChange.txt')
df2_Straight= pd.read_csv('Track_Sensor_KF_Straight.txt')

df3_circle = pd.read_csv('Track_Sensor_KFNO_Circle.txt')
df3_Lanechange = pd.read_csv('Track_Sensor_KFNO_LaneChange.txt')
df3_Straight= pd.read_csv('Track_Sensor_KFNO_Straight.txt')

df4_circle = pd.read_csv('Track_Sensor_SHARF_Circle.txt')
df4_Lanechange = pd.read_csv('Track_Sensor_SHARF_LaneChange.txt')
df4_Straight= pd.read_csv('Track_Sensor_SHARF_Straight.txt')

# df_circle = pd.append(df1_Circle,df2_Circle,df3_Circle,df4_Circle)
# df_Lanechange = pd.append(df1_Lanechange,df2_Lanechange,df3_Lanechange,df4_Lanechange)
# df_Straight = pd.append(df1_Straight,df2_Straight,df3_Straight,df4_Straight)

##################################################
#### PLOT TRACK FOR EACH ALGORITHM   #############
##################################################

fig, ax = plt.subplots(figsize=(5,6))
plt.plot(df1_circle['Ground Truth X'].to_numpy(),df1_circle['Ground Truth Y'].to_numpy(),color='k',label="Ground Truth")
plt.plot(df1_circle['IMU X'].to_numpy(),df1_circle['IMU Y'].to_numpy(),color='tab:red',linestyle = '-',linewidth=3,label="Local Estimator")
plt.plot(df1_circle['GPS X'].to_numpy(),df1_circle['GPS Y'].to_numpy(),color='#f37651',linestyle = '-',linewidth=4,label="GPS Spoofed")
plt.plot(df4_circle['Filter X'].to_numpy(),df4_circle['Filter Y'].to_numpy(),color='#35193e',linestyle = 'dotted',linewidth=2,label="SHARF")
plt.plot(df2_circle['Filter X'].to_numpy(),df2_circle['Filter Y'].to_numpy(),color = '#701f57',linestyle= '--',label="KF W Chi-Sqaured")
plt.plot(df3_circle['Filter X'].to_numpy(),df3_circle['Filter Y'].to_numpy(),color = '#e13342',linestyle= 'dotted',label="Naive KF")
plt.plot(df1_circle['Filter X'].to_numpy(),df1_circle['Filter Y'].to_numpy(),color = '#f6b48f',linestyle= 'dotted',label="CI W Chi-Squared")
plt.xlabel('South Direction (m)')
plt.ylabel('East Direction (m)')
plt.legend(loc='upper right')
plt.savefig("TrackCircle.pdf", format="pdf", bbox_inches="tight")
plt.show()

fig, ax = plt.subplots(figsize=(5,6))
plt.plot(df1_Lanechange['Ground Truth X'].to_numpy(),df1_Lanechange['Ground Truth Y'].to_numpy(),color='k',label="Ground Truth")
plt.plot(df1_Lanechange['IMU X'].to_numpy(),df1_Lanechange['IMU Y'].to_numpy(),color='tab:red',linestyle = '-',linewidth=3,label="Local Estimator")
plt.plot(df1_Lanechange['GPS X'].to_numpy(),df1_Lanechange['GPS Y'].to_numpy(),color='#f37651',linestyle = '-',linewidth=4,label="GPS Spoofed")
plt.plot(df4_Lanechange['Filter X'].to_numpy(),df4_Lanechange['Filter Y'].to_numpy(),color='#35193e',linestyle = 'dotted',linewidth=2,label="SHARF")
plt.plot(df2_Lanechange['Filter X'].to_numpy(),df2_Lanechange['Filter Y'].to_numpy(),color = '#701f57',linestyle= '--',label="KF W Chi-Sqaured")
plt.plot(df3_Lanechange['Filter X'].to_numpy(),df3_Lanechange['Filter Y'].to_numpy(),color = '#e13342',linestyle= 'dotted',label="Naive KF")
plt.plot(df1_Lanechange['Filter X'].to_numpy(),df1_Lanechange['Filter Y'].to_numpy(),color = '#f6b48f',linestyle= 'dotted',label="CI W Chi-Squared")
plt.xlabel('South Direction (m)')
plt.ylabel('East Direction (m)')
plt.legend(loc='upper right')
plt.savefig("TrackLanechange.pdf", format="pdf", bbox_inches="tight")
plt.show()

fig, ax = plt.subplots(figsize=(5,6))
plt.plot(df1_Straight['Ground Truth X'].to_numpy(),df1_Straight['Ground Truth Y'].to_numpy(),color='k',label="Ground Truth")
plt.plot(df1_Straight['IMU X'].to_numpy(),df1_Straight['IMU Y'].to_numpy(),color='tab:red',linestyle = '-',linewidth=3,label="Local Estimator")
plt.plot(df1_Straight['GPS X'].to_numpy(),df1_Straight['GPS Y'].to_numpy(),color='#f37651',linestyle = '-',linewidth=4,label="GPS Spoofed")
plt.plot(df4_Straight['Filter X'].to_numpy(),df4_Straight['Filter Y'].to_numpy(),color='#35193e',linestyle = 'dotted',linewidth=2,label="SHARF")
plt.plot(df2_Straight['Filter X'].to_numpy(),df2_Straight['Filter Y'].to_numpy(),color = '#701f57',linestyle= '--',label="KF W Chi-Sqaured")
plt.plot(df3_Straight['Filter X'].to_numpy(),df3_Straight['Filter Y'].to_numpy(),color = '#e13342',linestyle= 'dotted',label="Naive KF")
plt.plot(df1_Straight['Filter X'].to_numpy(),df1_Straight['Filter Y'].to_numpy(),color = '#f6b48f',linestyle= 'dotted',label="CI W Chi-Squared")
plt.xlabel('South Direction (m)')
plt.ylabel('East Direction (m)')
plt.legend(loc='upper right')
plt.savefig("TrackStraight.pdf", format="pdf", bbox_inches="tight")
plt.show()



