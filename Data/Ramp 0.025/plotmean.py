import matplotlib.pyplot as plt
import csv
import numpy as np
import pandas as pd
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

row_list = []
row_list2 = []

##### OPENING SHARF DATA ####
path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/SHARF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
      #t = -float(row[2])+t1_0
      total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
      total_error_1f.append(total_error)
         
      dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", Scenario = "Circle",AverageHealth = 0.972)#978
      row_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "SHARF", Scenario = "Lane_change",AverageHealth = 0.972)#967
         row_list.append(dict1)


with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", Scenario = "Straight",AverageHealth = 0.970)#980
         row_list.append(dict1)





############################
#### OPENING KF DATA ####
############################


path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/KF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", Scenario = "Circle",AverageHealth = 0.972)#978
         row_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "KF W Chi-Squared", Scenario = "Lane_change",AverageHealth = 0.972)#967
         row_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", Scenario = "Straight",AverageHealth = 0.970)#980
         row_list.append(dict1)
 
##################################################
######### Opening KF withot CHi         
path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/KF_no_detector"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", Scenario = "Circle",AverageHealth = 0.972)#978
         row_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "Naive KF", Scenario = "Lane_change",AverageHealth = 0.972)#967
         row_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", Scenario = "Straight",AverageHealth = 0.970)#980
         row_list.append(dict1)
         


############################
#### OPENING CI DATA ####
############################

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/CI"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", Scenario = "Circle",AverageHealth = 0.972)#978
         row_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "CI W Chi-Squared", Scenario = "Lane_change",AverageHealth = 0.972)#967
         row_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", Scenario = "Straight",AverageHealth = 0.970)#980
         row_list.append(dict1)




df = pd.DataFrame(row_list, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario'])
# df2 = pd.DataFrame(row_list2, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario'])
print(df)
# sns.set(rc={'figure.figsize':(7,3)})

# sns.set_theme(style="whitegrid")
# sns.color_palette("Spectral", as_cmap=True)
fig, ax = plt.subplots(figsize=(7, 3))
color_arr =  plt.cm.cool(np.linspace(0, 1, 5))

sns.boxplot(y = "TotalError",
            x = "Scenario",
            hue = "Algorithm",data =df, palette="rocket",showfliers=False,showmeans=True)

plt.show()
# fig, ax = plt.subplots(figsize=(5, 3))
# sns.boxplot(y = "TotalError",
#             x = "Scenario",
#             hue = "Algorithm", data =df2,palette="tab10",showfliers=False,showmeans=True)

# plt.show()

# fig, ax = plt.subplots(figsize=(7,4))
# sns.set_theme(style="whitegrid")
# sns.barplot(df, x="AverageHealth", y="TotalError",hue="Scenario",palette="rocket", edgecolor="k",errorbar=None,width=0.6)
# # sns.catplot(data=df, x="AverageHealth", y="TotalError", col="Track",kind='bar' ,palette="Set2" ,errorbar=None)
plt.show()





