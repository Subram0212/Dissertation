import pandas as pd
import numpy as np
import math


def euclidean_distance(position_1, position_2):  # This function calculates the distance UAV travels from one mission point to another
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])), 2)


def distance(position_1, position_2):
    df = pd.read_excel('ARL corridor points ordered.xlsx')
    # df = pd.read_csv('Hardware experimental scaled up scenario in miles ordered.csv')
    # df = pd.read_excel('Mission locations hardware - ordered.xlsx')
    df_list = df.values.tolist()
    ugv_distance = 0
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    # print(df_list)
    # for i, pt in enumerate(df_list):
    #     if pt == (3.02, 8.87):
    #         df_list[i] = (3.01, 8.87)
    #     elif pt == (3.1, 7.23):
    #         df_list[i] = (3.1, 7.24)
    #     elif pt == (10.75, 1.78):
    #         df_list[i] = (10.75, 1.79)
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
    df = pd.read_excel('ARL corridor points ordered DepotB.xlsx')
    # df = pd.read_excel('Mission locations hardware - ordered DepotB.xlsx')
    # df = pd.read_csv('Hardware experimental scaled up scenario in miles depotb ordered.csv')
    df_list = df.values.tolist()
    ugv_distance = 0
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    # print(df_list)
    count_1 = None
    count_2 = None
    # for i, pt in enumerate(df_list):
    #     if pt == (3.02, 8.87):
    #         df_list[i] = (3.01, 8.87)
    #     elif pt == (3.1, 7.23):
    #         df_list[i] = (3.1, 7.24)
    #     elif pt == (10.75, 1.78):
    #         df_list[i] = (10.75, 1.79)
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


def distance_btw_ugvstops(position_1, position_2):
    df = pd.read_excel('ARL corridor points ordered.xlsx')
    # df = pd.read_csv('Hardware experimental scaled up scenario in miles ordered.csv')
    # df = pd.read_excel('Mission locations hardware - ordered.xlsx')
    df_list = df.values.tolist()
    ugv_distance = 0
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    # print(df_list)
    # for i, pt in enumerate(df_list):
    #     if pt == (3.02, 8.87):
    #         df_list[i] = (3.01, 8.87)
    #     elif pt == (3.1, 7.23):
    #         df_list[i] = (3.1, 7.24)
    #     elif pt == (10.75, 1.78):
    #         df_list[i] = (10.75, 1.79)
    count_1 = None
    count_2 = None
    for i, list_val in enumerate(df_list):
        if df_list[i] == (round(position_1[0], 2), round(position_1[1], 2)):
            count_1 = i
        if df_list[i] == (round(position_2[0], 2), round(position_2[1], 2)):
            count_2 = i
    if count_1 < count_2:
        for j in range(count_1, count_2):
            # print(df_list[j], df_list[j+1])
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    elif count_2 < count_1:
        for j in range(count_2, count_1):
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    return round(ugv_distance*5280)


def distance_from_depotB_btw_ugvstops(position_1, position_2):
    df = pd.read_excel('ARL corridor points ordered DepotB.xlsx')
    # df = pd.read_csv('Hardware experimental scaled up scenario in miles depotb ordered.csv')
    # df = pd.read_excel('Mission locations hardware - ordered DepotB.xlsx')
    df_list = df.values.tolist()
    ugv_distance = 0
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    # print(df_list)
    count_1 = None
    count_2 = None
    # for i, pt in enumerate(df_list):
    #     if pt == (3.02, 8.87):
    #         df_list[i] = (3.01, 8.87)
    #     elif pt == (3.1, 7.23):
    #         df_list[i] = (3.1, 7.24)
    #     elif pt == (10.75, 1.78):
    #         df_list[i] = (10.75, 1.79)
    for i, list_val in enumerate(df_list):
        if df_list[i] == (round(position_1[0], 2), round(position_1[1], 2)):
            count_1 = i
        if df_list[i] == (round(position_2[0], 2), round(position_2[1], 2)):
            count_2 = i
    if count_1 < count_2:
        for j in range(count_1, count_2):
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    elif count_2 < count_1:
        for j in range(count_2, count_1):
            ugv_distance += euclidean_distance(df_list[j], df_list[j+1])
    return round(ugv_distance*5280)


if __name__ == "__main__":
    chuma = distance_from_depotB((0.6*5280, 8.07*5280), (3.79*5280, 6.71*5280))
    print(chuma)
