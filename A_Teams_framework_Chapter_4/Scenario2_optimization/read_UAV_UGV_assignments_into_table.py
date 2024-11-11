import pandas as pd
import numpy as np
from scipy import interpolate

df = pd.read_excel('plotting A-teams optimized parameter data_scn2.xlsx')
# print(df)

df2 = pd.DataFrame(df)
# print(df2)

# df3 = pd.DataFrame()

df_list = df2.values.tolist()
# print(df_list)

new_list = [[i, 0, 0] for i in range(151)]
# print(new_list)

for j in range(len(df_list)):
    for k in range(len(new_list)):
        if new_list[k][0] == df_list[j][0]:
            new_list[k][1] = df_list[j][1]
            new_list[k][2] = df_list[j][2]
            # new_list[k][3] = df_list[j][3]
# print(new_list)

ugv_list = [[0, 0] for j in range(151)]
for i in range(len(ugv_list)):
    if i == 0:
        ugv_list[i] = [8.66, 5]
    if 26 <= i <= 32:
        ugv_list[i] = [5.77, 5.83]
    elif 52 <= i <= 86:
        ugv_list[i] = [5.35, 6.1]
    elif 92 <= i <= 123:
        ugv_list[i] = [5.77, 5.83]
    elif i == 138:
        ugv_list[i] = [7.21, 5.41]
    elif i == 151:
        ugv_list[i] = [8.66, 5]

df3 = pd.DataFrame(new_list, columns=['time', 'x', 'y'])
df4 = pd.DataFrame(ugv_list, columns=['x1', 'y1'])
# combine = [df3, df4]
df5 = pd.concat([df3, df4], axis=1, join='inner')
print(df5)

df5.to_excel('data_to_interpolate_scenario2_depotA start.xlsx', index=False, engine='openpyxl')

# interpolation
dataframe_to_numpy = df3.to_numpy(dtype=float)
# print(dataframe_to_numpy)

# x = [dataframe_to_numpy[1][1], dataframe_to_numpy[4][1]]
# y = [dataframe_to_numpy[1][2], dataframe_to_numpy[4][2]]
# f = interpolate.interp1d(x, y)
# print(f)



# df_new = pd.DataFrame(np.insert(df.values, [x -1 for x in range(358)], values=list(df['time'].values()), axis=0), columns=df['time'])
# print(df_new)
# for _ in range(58, 358):
#     df2 = df2.append(pd.Series(), ignore_index=True)
#
# clock_time = [i for i in range(358)]
# df2.insert(0, 'elapsed_time', clock_time)
# print(df2)

#
# for j in range(df2[1]):
#     for k in range(len(clock_time)):
#         if j == k:
#             temp = df2[1][j]
#             df2[1][j] = df2[1][-1]
#             df2[1][k] = temp
