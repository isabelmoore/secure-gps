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

row_list = []
row_list2 = []
row_list3 = []
KF_list =[]
SHARF_list=[]
KFNO_list =[]
CI_list=[]
KF_list_0 =[]
SHARF_list_0=[]
KFNO_list_0 =[]
CI_list_0=[]
##### OPENING SHARF DATA ####
path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0/SHARF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
      #t = -float(row[2])+t1_0
      total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
      total_error_1f.append(total_error)
         
      dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 0,Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
      row_list.append(dict1)
      SHARF_list_0.append(dict1)


with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "SHARF", RampRate = 0, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         SHARF_list_0.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 0, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         SHARF_list_0.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/SHARF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
      #t = -float(row[2])+t1_0
      total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
      total_error_1f.append(total_error)
         
      dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 0.5,Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
      row_list.append(dict1)
      SHARF_list.append(dict1)


with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "SHARF", RampRate = 0.5, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         SHARF_list.append(dict1)


with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 0.5, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         SHARF_list.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.1/SHARF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
      #t = -float(row[2])+t1_0
      total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
      total_error_1f.append(total_error)
         
      dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 2,Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
      row_list.append(dict1)
      SHARF_list.append(dict1)


with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "SHARF", RampRate = 2, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         SHARF_list.append(dict1)


with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 2, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         SHARF_list.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.05/SHARF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
      #t = -float(row[2])+t1_0
      total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
      total_error_1f.append(total_error)
         
      dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 1,Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
      row_list.append(dict1)
      SHARF_list.append(dict1)


with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "SHARF", RampRate = 1, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         SHARF_list.append(dict1)


with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "SHARF", RampRate = 1, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         SHARF_list.append(dict1)




############################
#### OPENING KF DATA ####
############################
path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0/KF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 0, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KF_list_0.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "KF W Chi-Squared", RampRate = 0, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KF_list_0.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 0, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KF_list_0.append(dict1)



path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/KF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 0.5, Scenario = "Circle",AverageHealth =math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KF_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "KF W Chi-Squared", RampRate = 0.5, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KF_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 0.5, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KF_list.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.1/KF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 2, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KF_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "KF W Chi-Squared", RampRate = 2, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KF_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 2, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KF_list.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.05/KF"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 1, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KF_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "KF W Chi-Squared", RampRate = 1, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KF_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "KF W Chi-Squared", RampRate = 1, Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KF_list.append(dict1)

##################################################
######### Opening KF withot CHi       

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0/KF_no_detector"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 0, Scenario = "Circle",AverageHealth =math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KFNO_list_0.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "Naive KF", RampRate = 0, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KFNO_list_0.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 0,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KFNO_list_0.append(dict1)


path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/KF_no_detector"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 0.5, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KFNO_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "Naive KF", RampRate = 0.5, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KFNO_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 0.5,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KFNO_list.append(dict1)
         
path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.05/KF_no_detector"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 1, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KFNO_list.append(dict1)

with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "Naive KF", RampRate = 1, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KFNO_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 1,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KFNO_list.append(dict1)
         
path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.1/KF_no_detector"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 2, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         KFNO_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "Naive KF", RampRate = 2, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         KFNO_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "Naive KF", RampRate = 2,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         KFNO_list.append(dict1)
         


############################
#### OPENING CI DATA ####
############################

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0/CI"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 0, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         CI_list_0.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "CI W Chi-Squared", RampRate = 0, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         CI_list_0.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 0,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         CI_list_0.append(dict1)


path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.025/CI"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 0.5, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         CI_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "CI W Chi-Squared", RampRate = 0.5, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         CI_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 0.5,  Scenario = "Straight",AverageHealth =math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         CI_list.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.05/CI"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 1, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         CI_list.append(dict1)



with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "CI W Chi-Squared", RampRate = 1, Scenario = "Lane_change",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         CI_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 1,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         CI_list.append(dict1)

path = "/home/iea/IEA_Demo/MKZ_SIMULATOR_PROTYPE1/Data/Ramp 0.1/CI"
os.chdir(path)
print(os.getcwd())
with open('Track_circle_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
	 
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         total_error_1f.append(total_error)
         
         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 2, Scenario = "Circle",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#978
         row_list.append(dict1)
         CI_list.append(dict1)

with open('Track_lanechange_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:

         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)
         dict1 = dict(TotalError = total_error, Track = 2 , Algorithm = "CI W Chi-Squared", RampRate = 2, Scenario = "Lane_change",AverageHealth =math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#967
         row_list2.append(dict1)
         CI_list.append(dict1)

with open('Track_straight_spoof.txt','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
         total_error = np.sqrt(float(row[2])**2+float(row[3])**2)

         dict1 = dict(TotalError = total_error, Track = 1, Algorithm = "CI W Chi-Squared", RampRate = 2,  Scenario = "Straight",AverageHealth = math.floor(10*float(row[5]))/10,GPSHealth = math.floor(10*float(row[6]))/10)#980
         row_list3.append(dict1)
         CI_list.append(dict1)



df = pd.DataFrame(row_list, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate'])
df2 = pd.DataFrame(row_list2, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate'])
df3 = pd.DataFrame(row_list3, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate'])

df_KF = pd.DataFrame(KF_list, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])
df_CI = pd.DataFrame(CI_list, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])
df_KFNo = pd.DataFrame(KFNO_list, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])
df_SHARF = pd.DataFrame(SHARF_list, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])

df_KF_0 = pd.DataFrame(KF_list_0, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])
df_CI_0 = pd.DataFrame(CI_list_0, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])
df_KFNo_0 = pd.DataFrame(KFNO_list_0, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])
df_SHARF_0 = pd.DataFrame(SHARF_list_0, columns=['TotalError','Track','AverageHealth','Algorithm','Scenario','RampRate','GPSHealth'])


mean_SHARF = df_SHARF.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_CI = df_CI.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_KF = df_KF.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_KFNO = df_KFNo.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_SHARF_0 = df_SHARF_0.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_CI_0 = df_CI_0.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_KF_0 = df_KF_0.groupby('GPSHealth')['TotalError'].mean().to_numpy()
mean_KFNO_0 = df_KFNo_0.groupby('GPSHealth')['TotalError'].mean().to_numpy()
color_arr_0 =  plt.cm.cool(np.linspace(0, 1, 5))
# palette = ["#F72585", "#7209B7", "#3A0CA3", "#4361EE", "#4CC9F0"]

# sns.boxplot(y = "TotalError",
#             x = "RampRate",
#             hue = "Algorithm",data =df, palette="rocket",showfliers=False,showmeans=True)

# plt.show()
# sns.boxplot(y = "TotalError",
#             x = "RampRate",
#             hue = "Algorithm",data =df2, palette="rocket",showfliers=False,showmeans=True)

# plt.show()
# sns.boxplot(y = "TotalError",
#             x = "RampRate",
#             hue = "Algorithm",data =df3, palette="rocket",showfliers=False,showmeans=True)

# plt.show()

# sns.boxplot(y = "TotalError",
#             x = "GPSHealth",
#             hue = "Algorithm",data =df3, palette="rocket",showfliers=False,showmeans=True)

# plt.show()
pal = sns.color_palette("rocket")
print(pal.as_hex()[:])
# ['#35193e', '#701f57', '#ad1759', '#e13342', '#f37651', '#f6b48f']
pal.as_hex()
x_plot = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
# sns.set_palette(palette)
# fig, ax = plt.subplots(figsize=(6,4))
plt.plot(x_plot,mean_SHARF,marker='p',color='#35193e',label='SHARF',linestyle ='--')
plt.plot(x_plot,mean_KF,marker='p',color= '#701f57',label='KF W Chi-Squared',linestyle ='--')
plt.plot(x_plot,mean_KFNO,marker='p',color='#e13342',label='Naive KF',linestyle ='--')
plt.plot(x_plot,mean_CI,marker='p',color='#f6b48f',label='CI W Chi-Squared',linestyle ='--')

plt.plot(x_plot,mean_SHARF_0,marker='p',color='#35193e',label='SHARF')
plt.plot(x_plot,mean_KF_0,marker='p',color= '#701f57',label='KF W Chi-Squared')
plt.plot(x_plot,mean_KFNO_0,marker='p',color='#e13342',label='Naive KF')
plt.plot(x_plot,mean_CI_0,marker='p',color='#f6b48f',label='CI W Chi-Squared')


plt.xlabel('Health Status of Spoofed Signal')
plt.ylabel('Mean of Total Error')
plt.grid(color='0.8', linestyle='-', linewidth=0.1)
legend_elements= [Patch(facecolor='#35193e', edgecolor='k',label='SHARF'),
                  Patch(facecolor='#701f57', edgecolor='k',label='KF W Chi-Squared'),
                  Patch(facecolor='#e13342', edgecolor='k',label='Naive KF'),
                  Patch(facecolor='#f6b48f', edgecolor='k',label='CI W Chi-Squared'),
                       Line2D([0],[0],color='k',linestyle='-',label= 'avg Health >= 95%',),
                       Line2D([0],[0],color='k',linestyle='--', label= 'avg Health < 95%')]
plt.legend(handles= legend_elements ,loc='upper right',frameon= True, prop={'size': 7})
plt.show()





