import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns



'''Testing'''

# df = pd.read_csv('health_data.txt')

# time_total = (df['Time Step'].astype(str).str.replace('[', '').str.replace(']', '').astype(float))
# time = ((time_total - time_total.iloc[0])/ 10**9).tolist()
# health = (df['Health Status'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
# dm = (df['dm'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
# dmdt = (df['dm / dt'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()


# df = pd.read_csv('ground_truth_data.txt')
# diff_x = (df['Filter Difference X'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
# diff_y = (df['Filter Difference Y'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

# print(f'X-Error Mean {pd.Series(diff_x).mean()}')
# print(f'X-Error STD {pd.Series(diff_x).std()}')

# print(f'Y-Error Mean {pd.Series(diff_y).mean()}')
# print(f'X-Error STD {pd.Series(diff_y).std()}')

# color_arr =  plt.cm.cool(np.linspace(0, 1, 4))

# plt.figure(figsize=(7, 4))
# plt.scatter(time, dm, color=color_arr[2], s=4)
# plt.title('Filter Position Change')
# plt.xlabel('Time (s)')
# plt.ylabel('Distance (m)')
# plt.grid(True)
# plt.tight_layout()
# plt.savefig('pos.png')

# plt.figure(figsize=(7, 4))
# plt.scatter(time, dmdt, color=color_arr[2], s=4)
# plt.title('Filter Velocity Change')
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# plt.grid(True)
# plt.tight_layout()
# plt.savefig('vel.png')

# plt.figure(figsize=(7, 4))
# plt.scatter(time, health, color=color_arr[2], s=4)
# plt.title('Filter Health')
# plt.xlabel('Time (s)')
# plt.ylabel('Health')
# plt.grid(True)
# plt.tight_layout()
# plt.savefig('health.png')

# plt.figure(figsize=(7, 4))
# plt.scatter(range(len(diff_x)), diff_x, color=color_arr[0], s=4)
# plt.title('X-Position Error')
# plt.ylabel('Distance (m)')
# plt.grid(True)
# plt.tight_layout()
# plt.savefig('xpos_error.png')

# plt.figure(figsize=(7, 4))
# plt.scatter(range(len(diff_y)), diff_y, color=color_arr[1], s=4)
# plt.title('Y-Position Error')
# plt.ylabel('Distance (m)')
# plt.grid(True)
# plt.tight_layout()
# plt.savefig('ypos_error.png')


# df = pd.DataFrame({'X-Position Error': diff_x, 'Y-Position Error': diff_y})
# df_melted = df.melt(var_name='Coordinates', value_name='Distance (m)')
# plt.figure(figsize=(7, 4))
# sns.boxplot(x='Coordinates', y='Distance (m)', data=df_melted, palette=[color_arr[0], color_arr[1]], showfliers=False)
# plt.title('Box Plot of X and Y Position Errors')
# plt.grid(True)
# plt.tight_layout()
# plt.savefig('box.png')
# plt.show()




'''Comaprison'''

label1 = 'True with HM'
label2 = 'Spoof with HM'
label3 = 'True without HM'
label4 = 'Spoof without HM'


df = pd.read_csv('health_data_WHM_true.txt')
time_total = (df['Time Step'].astype(str).str.replace('[', '').str.replace(']', '').astype(float))
time_whm_norm = ((time_total - time_total.iloc[0])/ 10**9).tolist()
health_whm_norm = (df['Health Status'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dm_whm_norm = (df['dm'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dmdt_whm_norm = (df['dmdt'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('health_data_WHM_spoof.txt')
time_total = (df['Time Step'].astype(str).str.replace('[', '').str.replace(']', '').astype(float))
time_whm_spoof = ((time_total - time_total.iloc[0])/ 10**9).tolist()
health_whm_spoof = (df['Health Status'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dm_whm_spoof = (df['dm'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dmdt_whm_spoof = (df['dmdt'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('health_data_WoHM_true.txt')
time_total = (df['Time Step'].astype(str).str.replace('[', '').str.replace(']', '').astype(float))
time_wohm_norm = ((time_total - time_total.iloc[0])/ 10**9).tolist()
health_wohm_norm = (df['Health Status'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dm_wohm_norm = (df['dm'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dmdt_wohm_norm = (df['dmdt'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('health_data.txt')
time_total = (df['Time Step'].astype(str).str.replace('[', '').str.replace(']', '').astype(float))
time_wohm_spoof = ((time_total - time_total.iloc[0])/ 10**9).tolist()
health_wohm_spoof = (df['Health Status'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dm_wohm_spoof = (df['dm'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
dmdt_wohm_spoof = (df['dmdt'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('ground_truth_data_WHM_true.txt')
diff_x_whm_norm = (df['Filter Difference X'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
diff_y_whm_norm = (df['Filter Difference Y'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('ground_truth_data_WHM_spoof.txt')
diff_x_whm_spoof = (df['Filter Difference X'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
diff_y_whm_spoof = (df['Filter Difference Y'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('ground_truth_data_WoHM_true.txt')
diff_x_wohm_norm = (df['Filter Difference X'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
diff_y_wohm_norm = (df['Filter Difference Y'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

df = pd.read_csv('ground_truth_data.txt')
diff_x_wohm_spoof = (df['Filter Difference X'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()
diff_y_wohm_spoof = (df['Filter Difference Y'].astype(str).str.replace('[', '').str.replace(']', '').astype(float)).tolist()

# print(f'X-Error Mean {pd.Series(diff_x).mean()}')
# print(f'X-Error STD {pd.Series(diff_x).std()}')

# print(f'Y-Error Mean {pd.Series(diff_y).mean()}')
# print(f'X-Error STD {pd.Series(diff_y).std()}')


color_arr =  plt.cm.cool(np.linspace(0, 1, 5))

# Filter Position Change
plt.figure(figsize=(7, 4))
plt.scatter(time_whm_norm, dm_whm_norm, color=color_arr[0], label=label1, s=4)
plt.scatter(time_whm_spoof, dm_whm_spoof, color=color_arr[2], label=label2, s=4)
plt.scatter(time_wohm_norm, dm_wohm_norm, color=color_arr[1], label=label3, s=4)
plt.scatter(time_wohm_spoof, dm_wohm_spoof, color=color_arr[3], label=label4, s=4)
plt.title('Filter Position Change')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.grid(True)
plt.tight_layout()
plt.legend()
plt.savefig('pos_all.png')

# Filter Velocity Change
plt.figure(figsize=(7, 4))
# plt.scatter(time_whm_norm, dmdt_whm_norm, color=color_arr[0], label=label1, s=4)
# plt.scatter(time_whm_spoof, dmdt_whm_spoof, color=color_arr[2], label=label2, s=4)
# plt.scatter(time_wohm_norm, dmdt_wohm_norm, color=color_arr[1], label=label3, s=4)
plt.scatter(time_wohm_spoof, dmdt_wohm_spoof, color=color_arr[3], label=label4, s=4)
plt.title('Filter Velocity Change')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)
plt.tight_layout()
plt.legend()
plt.savefig('vel_all.png')

# Health Status Over Time
plt.figure(figsize=(7, 4))
plt.scatter(time_whm_norm, health_whm_norm, color=color_arr[0], label=label1, s=4)
plt.scatter(time_whm_spoof, health_whm_spoof, color=color_arr[2], label=label2, s=4)
plt.scatter(time_wohm_norm, health_wohm_norm, color=color_arr[1], label=label3, s=4)
plt.scatter(time_wohm_spoof, health_wohm_spoof, color=color_arr[3], label=label4, s=4)
plt.xlabel('Time (s)')
plt.ylabel('Health')
plt.title('Health Status')
plt.grid(True)
plt.tight_layout()
plt.legend()
plt.savefig('health_all.png')


df_health = pd.DataFrame({
    f'{label1}': pd.Series(health_whm_norm),
    f'{label2}': pd.Series(health_whm_spoof),
    f'{label3}': pd.Series(health_wohm_norm),
    f'{label4}': pd.Series(health_wohm_spoof),
})

min_length = df_health.count().min()
df_health = df_health.apply(lambda x: x[:min_length])
df_melted = df_health.melt(var_name=' ', value_name='Health')
plt.figure(figsize=(8, 4))
sns.boxplot(x=' ', y='Health', data=df_melted, palette=[color_arr[0], color_arr[2]], showfliers=False)
plt.grid(True)
plt.title('Health Status')
plt.tight_layout()
plt.savefig('health_box.png')

# X-Position Error
plt.figure(figsize=(7, 4))
plt.scatter(range(len(diff_x_whm_norm)), diff_x_whm_norm, color=color_arr[0], label=label1, s=4)
plt.scatter(range(len(diff_x_whm_spoof)), diff_x_whm_spoof, color=color_arr[2], label=label2, s=4)
plt.scatter(range(len(diff_x_wohm_norm)), diff_x_wohm_norm, color=color_arr[1], label=label3, s=4)
plt.scatter(range(len(diff_x_wohm_spoof)), diff_x_wohm_spoof, color=color_arr[3], label=label4, s=4)
plt.title('X-Position Error')
plt.ylabel('Distance (m)')
plt.grid(True)
plt.tight_layout()
plt.legend()
plt.savefig('xpos_error_all.png')

# Y-Position Error
plt.figure(figsize=(7, 4))
plt.scatter(range(len(diff_y_whm_norm)), diff_y_whm_norm, color=color_arr[0], label=label1, s=4)
plt.scatter(range(len(diff_y_whm_spoof)), diff_y_whm_spoof, color=color_arr[2], label=label2, s=4)
plt.scatter(range(len(diff_y_wohm_norm)), diff_y_wohm_norm, color=color_arr[1], label=label3, s=4)
plt.scatter(range(len(diff_y_wohm_spoof)), diff_y_wohm_spoof, color=color_arr[3], label=label4, s=4)
plt.title('Y-Position Error')
plt.ylabel('Distance (m)')
plt.grid(True)
plt.tight_layout()
plt.legend()
plt.savefig('ypos_error_all.png')


# Box Plot for Health Status
label_whm_norm = label1
label_whm_spoof = label2
label_wohm_norm = label3
label_wohm_spoof = label4

df_x_total = pd.DataFrame({
    f'X-Pos {label_whm_norm}': pd.Series(diff_x_whm_norm),
    f'X-Pos {label_whm_spoof}': pd.Series(diff_x_whm_spoof),
    f'X-Pos {label_wohm_norm}': pd.Series(diff_x_wohm_norm),
    f'X-Pos {label_wohm_spoof}': pd.Series(diff_x_wohm_spoof),
})

min_length = df_x_total.count().min()
df_x_total = df_x_total.apply(lambda x: x[:min_length])
df_melted = df_x_total.melt(var_name='Condition', value_name='Distance (m)')
plt.figure(figsize=(8, 4))
sns.boxplot(x='Condition', y='Distance (m)', data=df_melted, palette=[color_arr[0], color_arr[2], color_arr[1], color_arr[3]], showfliers=False)  # 'Set3' for a colorful palette
plt.title('X Position Error Across Conditions')
# plt.setp(plt.gca().get_xticklabels(), rotation_mode="anchor", wrap=True)
plt.grid(True)
plt.tight_layout()
plt.savefig('x_pos_errors_all_conditions.png')
plt.legend()


df_y_total = pd.DataFrame({
    f'Y-Pos {label_whm_norm}': pd.Series(diff_y_whm_norm),
    f'Y-Pos {label_whm_spoof}': pd.Series(diff_y_whm_spoof),
    f'Y-Pos {label_wohm_norm}': pd.Series(diff_y_wohm_norm),
    f'Y-Pos {label_wohm_spoof}': pd.Series(diff_y_wohm_spoof),
})

min_length = df_y_total.count().min()
df_y_total = df_y_total.apply(lambda x: x[:min_length])
df_melted = df_y_total.melt(var_name='Condition', value_name='Distance (m)')
plt.figure(figsize=(8, 4))
sns.boxplot(x='Condition', y='Distance (m)', data=df_melted, palette=[color_arr[0], color_arr[2], color_arr[1], color_arr[3]], showfliers=False)  # 'Set3' for a colorful palette
plt.title('Y Position Error Across Conditions')
# plt.setp(plt.gca().get_xticklabels(), rotation_mode="anchor", wrap=True)
plt.grid(True)
plt.tight_layout()
plt.savefig('y_pos_errors_all_conditions.png')
plt.legend()
plt.show()

