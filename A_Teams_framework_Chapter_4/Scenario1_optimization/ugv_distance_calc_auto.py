import pandas as pd
import numpy as np
import math


def euclidean_distance(position_1, position_2):  # This function calculates the distance UAV travels from one mission point to another
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])), 2)


def distance(position_1, position_2):
    df = pd.read_excel('ARL corridor points ordered.xlsx', engine='openpyxl')
    df_list = df.values.tolist()
    ugv_distance = 0
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    print(df_list)
    count_1 = None
    count_2 = None
    for i, list_val in enumerate(df_list):
        if df_list[i] == (round(position_1[0]/5280, 2), round(position_1[1]/5280, 2)):
            count_1 = i
        if df_list[i] == (round(position_2[0]/5280, 2), round(position_2[1]/5280, 2)):
            count_2 = i
    if count_1 < count_2:
        for j in range(count_1, count_2):
            # print(df_list[j], df_list[j+1])
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    elif count_2 < count_1:
        for j in range(count_2, count_1):
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    return round(ugv_distance*5280)


def distance_from_depotB(position_1, position_2):
    df = pd.read_excel('ARL corridor points ordered DepotB.xlsx', engine='openpyxl')
    df_list = df.values.tolist()
    ugv_distance = 0
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    print(df_list)
    count_1 = None
    count_2 = None
    for i, list_val in enumerate(df_list):
        if df_list[i] == (round(position_1[0]/5280, 2), round(position_1[1]/5280, 2)):
            count_1 = i
        if df_list[i] == (round(position_2[0]/5280, 2), round(position_2[1]/5280, 2)):
            count_2 = i
    if count_1 < count_2:
        for j in range(count_1, count_2):
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    elif count_2 < count_1:
        for j in range(count_2, count_1):
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    return round(ugv_distance*5280)

#
# chuma = distance((46200, 14414.4), (55651.2, 4752))
# print(chuma)
