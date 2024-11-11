import pandas as pd
import numpy as np
from scipy import interpolate

df = pd.read_excel('Scenario2_data_for_animation.xlsx')
# print(df)

df2 = pd.DataFrame(df)
# print(df2)

# df3 = pd.DataFrame()

df_list = df2.values.tolist()
# print(df_list)

new_list = [[i, 0, 0] for i in range(237)]
# print(new_list)

for j in range(len(df_list)):
    for k in range(len(new_list)):
        if new_list[k][0] == df_list[j][0]:
            new_list[k][1] = df_list[j][1]
            new_list[k][2] = df_list[j][2]
            # new_list[k][3] = df_list[j][3]
# print(new_list)

ugv_list = [[0, 0] for j in range(237)]
for i in range(len(ugv_list)):
    if i == 0:
        ugv_list[i] = [0.001, 0.001]
    if 4 <= i <= 4:
        ugv_list[i] = [0.25, 0.57]
    elif 18 <= i <= 18:
        ugv_list[i] = [1.72, 2.33]
    elif 65 <= i <= 65:
        ugv_list[i] = [4.45, 3.85]
    elif 80 <= i <= 80:
        ugv_list[i] = [2.18, 2.59]
    elif 92 <= i <= 92:
        ugv_list[i] = [0.83, 2.33]
    elif 96 <= i <= 96:
        ugv_list[i] = [0.9, 4.87]
    elif 113 <= i <= 136:
        ugv_list[i] = [0.93, 5.89]
    elif 150 <= i <= 150:
        ugv_list[i] = [0.86, 3.35]
    elif 165 <= i <= 167:
        ugv_list[i] = [1.72, 2.33]
    elif 182 <= i <= 182:
        ugv_list[i] = [4.0, 3.6]
    elif 185 <= i <= 206:
        ugv_list[i] = [4.45, 3.85]
    elif i == 236:
        ugv_list[i] = [0.001, 0.001]

df3 = pd.DataFrame(new_list, columns=['time', 'x', 'y'])
df4 = pd.DataFrame(ugv_list, columns=['x1', 'y1'])
# combine = [df3, df4]
df5 = pd.concat([df3, df4], axis=1, join='inner')
print(df5)

df5.to_excel('data_to_interpolate_scenario2_ensemble.xlsx', index=False, engine='openpyxl')

# interpolation
dataframe_to_numpy = df3.to_numpy(dtype=float)
# print(dataframe_to_numpy)
