import matplotlib.pyplot as plt
import csv
import numpy as np
import pandas as pd
import math
from matplotlib.lines import Line2D
from matplotlib.legend import Legend
from matplotlib.patches import Patch
import os
import seaborn as sns


t1_0 = 60


x1f=[]
y1f=[]
t1f=[]
H1f=[]
x2f=[]
y2f=[]
t2f=[]
H2f=[]

x1f3=[]
y1f3=[]
t1f3=[]
H1f3=[]
x2f3=[]
y2f3=[]
t2f3=[]
H2f3=[]

x1l=[]
y1l=[]
t1l=[]
H1l=[]
x2l=[]
y2l=[]
t2l=[]
H2l=[]

x1s=[]
y1s=[]
t1s=[]
H1s=[]
x2s=[]
y2s=[]
t2s=[]
H2s=[]

total_error_1f = []
total_error_2f = []
total_error_1f3 =[]
total_error_2f3 =[]
total_error_1l = []
total_error_2l = []
total_error_1s = []
total_error_2s = []

row_list1 = []
row_list2 = []
row_list3=[]
##### OPENING SHARF DATA ####

with open('lanechange_spoof_detail.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         dict1 = dict(Scenario = "LaneChange",GroundTruth_y = float(row[0]), GroundTruth_x =  float(row[1]), GPS_y =  float(row[2]), GPS_x =  float(row[3]), GPS_x_05 =  float(row[4]),GPS_x_1 =  float(row[5]),GPS_x_2 =  float(row[6]))
         row_list1.append(dict1)


with open('circle_spoof_detail.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         dict1 = dict(Scenario = "Circle",GroundTruth_y = float(row[0]), GroundTruth_x =  float(row[1]), GPS_y =  float(row[2]), GPS_x =  float(row[3]), GPS_x_05 =  float(row[4]),GPS_x_1 =  float(row[5]),GPS_x_2 =  float(row[6]))
         row_list2.append(dict1)


with open('straight_spoof_detail.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         dict1 = dict(Scenario = "Straight",GroundTruth_y = float(row[0]), GroundTruth_x =  float(row[1]), GPS_y =  float(row[2]), GPS_x =  float(row[3]), GPS_x_05 =  float(row[4]),GPS_x_1 =  float(row[5]),GPS_x_2 =  float(row[6]))
         row_list3.append(dict1)




df1 = pd.DataFrame(row_list1, columns=['GroundTruth_x','GroundTruth_y','GPS_y','GPS_x','GPS_x_05','GPS_x_1','GPS_x_2','Scenario'])
df2 = pd.DataFrame(row_list2, columns=['GroundTruth_x','GroundTruth_y','GPS_y','GPS_x','GPS_x_05','GPS_x_1','GPS_x_2','Scenario'])
df3 = pd.DataFrame(row_list3, columns=['GroundTruth_x','GroundTruth_y','GPS_y','GPS_x','GPS_x_05','GPS_x_1','GPS_x_2','Scenario'])
GR_x = df1['GroundTruth_x'].to_numpy()
GR_y =  df1['GroundTruth_y'].to_numpy()
fig, ax = plt.subplots(figsize=(5,6))
plt.plot(GR_x,GR_y,color='k',linewidth = 4,label="Ground Truth")
plt.plot(df1['GPS_x'].to_numpy(),df1['GPS_y'].to_numpy(),color='r',linestyle = 'dotted',label="GPS Not Spoofed")
plt.plot(df1['GPS_x_05'].to_numpy(),df1['GPS_y'].to_numpy(),color = 'r',linestyle= '-.',label="GPS Spoofed Rate 0.5 m/s")
plt.plot(df1['GPS_x_1'].to_numpy(),df1['GPS_y'].to_numpy(),color = 'r',linestyle= '--',label="GPS Spoofed Rate 1 m/s")
plt.plot(df1['GPS_x_2'].to_numpy(),df1['GPS_y'].to_numpy(),color = 'r',linestyle= '-',label="GPS Spoofed Rate 2 m/s")
plt.xlabel('South Direction (m)')
plt.ylabel('East Direction (m)')
plt.legend(loc='upper right')
plt.show()

fig, ax = plt.subplots(figsize=(5,6))

GR_x = df2['GroundTruth_x'].to_numpy()
GR_y =  df2['GroundTruth_y'].to_numpy()
plt.plot(GR_x,GR_y,color='k',linewidth = 4,label="Ground Truth")
plt.plot(df2['GPS_x'].to_numpy(),df2['GPS_y'].to_numpy(),color='r',linestyle = 'dotted',label="GPS Not Spoofed")
plt.plot(df2['GPS_x_05'].to_numpy(),df2['GPS_y'].to_numpy(),color = 'r',linestyle= '-.',label="GPS Spoofed Rate 0.5 m/s")
plt.plot(df2['GPS_x_1'].to_numpy(),df2['GPS_y'].to_numpy(),color = 'r',linestyle= '--',label="GPS Spoofed Rate 1 m/s")
plt.plot(df2['GPS_x_2'].to_numpy(),df2['GPS_y'].to_numpy(),color = 'r',linestyle= '-',label="GPS Spoofed Rate 2 m/s")
plt.xlabel('South Direction (m)')
plt.ylabel('East Direction (m)')
plt.legend(loc='upper right')
plt.show()
fig, ax = plt.subplots(figsize=(5,6))

GR_x = df3['GroundTruth_x'].to_numpy()
GR_y =  df3['GroundTruth_y'].to_numpy()
plt.plot(GR_x,GR_y,color='k',linewidth = 4,label="Ground Truth")
plt.plot(df3['GPS_x'].to_numpy(),df3['GPS_y'].to_numpy(),color='r',linestyle = 'dotted',label="GPS Not Spoofed")
plt.plot(df3['GPS_x_05'].to_numpy(),df3['GPS_y'].to_numpy(),color = 'r',linestyle= '-.',label="GPS Spoofed Rate 0.5 m/s")
plt.plot(df3['GPS_x_1'].to_numpy(),df3['GPS_y'].to_numpy(),color = 'r',linestyle= '--',label="GPS Spoofed Rate 1 m/s")
plt.plot(df3['GPS_x_2'].to_numpy(),df3['GPS_y'].to_numpy(),color = 'r',linestyle= '-',label="GPS Spoofed Rate 2 m/s")
plt.xlabel('South Direction (m)')
plt.ylabel('East Direction (m)')
plt.legend(loc='upper right')
plt.show()
# fig, ax = plt.subplots(figsize=(5, 3))

# sns.scatterplot(data=df1, x="GroundTruth_x", y="GroundTruth_y")
# sns.lineplot(data=df1, x="GPS_x", y="GPS_y")
# sns.lineplot(data=df1, x="GPS_x_05", y="GPS_y")
# sns.lineplot(data=df1, x="GPS_x_1", y="GPS_y")
# sns.lineplot(data=df1, x="GPS_x_2", y="GPS_y")
# plt.xlabel('South Direction (m)')
# plt.ylabel('East Direction (m)')
# legend_elements= [Patch(facecolor='tab:blue', edgecolor='k',label='SHARF'),
#                   Patch(facecolor='tab:green', edgecolor='k',label='CI'),
#                   Patch(facecolor='tab:orange', edgecolor='k',label='KF'),
#                        Line2D([0],[0],color='k',linestyle='-',label= 'avg Health >= 95%',),
#                        Line2D([0],[0],color='k',linestyle='--', label= 'avg Health < 95%')]
# plt.legend(handles= legend_elements ,loc='upper right',frameon= True, prop={'size': 7})
# plt.show()

# print(df)
# sns.set(rc={'figure.figsize':(7,3)})

# sns.set_theme(style="whitegrid")
# sns.color_palette("Spectral", as_cmap=True)
# fig, ax = plt.subplots(figsize=(5, 3))
# sns.boxplot(y = "TotalError",
#             x = "Scenario",
#             hue = "Algorithm",data =df,palette='Spectral', showfliers=False)

# plt.show()
# fig, ax = plt.subplots(figsize=(5, 3))
# sns.boxplot(y = "TotalError",
#             x = "Scenario",
#             hue = "Algorithm", data =df2,palette='mako', showfliers=False)
# mean_SHARF = df1.groupby('AverageHealth')['TotalError'].mean().to_numpy()
# mean_CI = df2.groupby('AverageHealth')['TotalError'].mean().to_numpy()
# mean_KF = df3.groupby('AverageHealth')['TotalError'].mean().to_numpy()
# mean_SHARF_ = df12.groupby('AverageHealth')['TotalError'].mean().to_numpy()
# mean_CI_ = df22.groupby('AverageHealth')['TotalError'].mean().to_numpy()
# mean_KF_ = df32.groupby('AverageHealth')['TotalError'].mean().to_numpy()

# # print("data Type is :",type(mean_SHARF),mean_SHARF)

# x_plot = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
# # plt.show()
# fig, ax = plt.subplots(figsize=(6,4))
# # ax.get_xaxis().set_visible(False)
# # sns.set_theme(style="whitegrid")
# # g2=sns.barplot(SHARF,  x="AverageHealth", y="TotalError",color='silver',errorbar=None,estimator='mean')
# plt.plot(x_plot,mean_SHARF,marker='p',color='tab:blue',label='SHARF')
# plt.plot(x_plot,mean_CI,marker='p',color='tab:orange',label='CI')
# plt.plot(x_plot,mean_KF,marker='p',color='tab:green',label='KF')
# plt.plot(x_plot,mean_SHARF_,marker='p',color='tab:blue',label='SHARF',linestyle ='--')
# plt.plot(x_plot,mean_CI_,marker='p',color='tab:orange',label='CI',linestyle ='--')
# plt.plot(x_plot,mean_KF_,marker='p',color='tab:green',label='KF',linestyle ='--')
# plt.xlabel('Instantaneous Average Health Status over all sensors')
# plt.ylabel('Mean of Total Error')
# plt.grid(color='0.8', linestyle='-', linewidth=0.1)
# legend_elements= [Patch(facecolor='tab:blue', edgecolor='k',label='SHARF'),
#                   Patch(facecolor='tab:green', edgecolor='k',label='CI'),
#                   Patch(facecolor='tab:orange', edgecolor='k',label='KF'),
#                        Line2D([0],[0],color='k',linestyle='-',label= 'avg Health >= 95%',),
#                        Line2D([0],[0],color='k',linestyle='--', label= 'avg Health < 95%')]
# plt.legend(handles= legend_elements ,loc='upper right',frameon= True, prop={'size': 7})
# g1=sns.barplot(df, x="AverageHealth", y="TotalError",color ="tab:blue",errorbar="sd",capsize=0.4,estimator='mean',edgecolor="k")
# g1=sns.barplot(df2, x="AverageHealth", y="TotalError",color ="tab:orange",errorbar="sd",capsize=0.4,estimator='mean',edgecolor="k")

# g1=sns.barplot(df3, x="AverageHealth", y="TotalError",color ="tab:green",errorbar="sd",capsize=0.4,estimator='mean',edgecolor="k")
# sns.catplot(data=df, x="AverageHealth", y="TotalError", col="Track",kind='bar' ,palette="Set2" ,errorbar=None)
# print(g1.get_xticks(),g1.get_xticklabels())
# g.set_xticks(my_x_ticks)
# g.set_xticklabels(my_x_label,rotation=45)

# colors = ['green', 'lightgreen', 'blue', 'lightblue', 'red'] ## Colors for the bars
# df.set_index('AverageHealth').plot(kind='bar', stacked=True, color=colors) ## Plot
# plt.ticklabel_format(style='plain', useOffset=False, axis='y') ## No offset
# plt.gca().set_ylabel("Total $'s")
# plt.xticks([0,7,14,21,28,35,42,48,54,60,65,71,73],rotation=0)
plt.show()





