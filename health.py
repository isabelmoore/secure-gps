import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns



'''Testing'''

df = pd.read_csv('health_data.txt')

time_total = (df['Time Step'].astype(str).str.replace('[', '').str.replace(']', '').astype(float))
time = ((time_total - time_total.iloc[0])/ 10**9).tolist()
health = (df['Health Status'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dm = (df['dm'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dmdt = (df['dmdt'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
domega = (df['domega'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

color_arr =  plt.cm.cool(np.linspace(0, 1, 4))

plt.figure(figsize=(7, 4))
plt.scatter(time, dm, color=color_arr[2], s=4)
plt.title('Filter Position Change')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(0,0.5)
plt.grid(True)
plt.tight_layout()

plt.savefig('pos.png')

plt.figure(figsize=(7, 4))
plt.scatter(time, dmdt, color=color_arr[2], s=4)
plt.title('Filter Velocity Change')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)
plt.tight_layout()
plt.savefig('vel.png')

plt.figure(figsize=(7, 4))
plt.scatter(time, domega, color=color_arr[2], s=4)
plt.title('Filter Angle Change')
plt.xlabel('Time (s)')
plt.ylabel('Angle (Rad)')
plt.ylim(-0.1,0.1)
plt.grid(True)
plt.tight_layout()

plt.savefig('angle.png')

plt.figure(figsize=(7, 4))
plt.scatter(time, health, color=color_arr[2], s=4)
plt.title('Filter Health')
plt.xlabel('Time (s)')
plt.ylabel('Health')
plt.grid(True)
plt.tight_layout()

plt.savefig('health.png')

