import pandas as pd
import numpy as np
from itertools import product, combinations
import copy
import math
import random


def scenario_1_data_processing(num_stops=2):
    df = pd.read_csv('Scenario 1 data points.csv')

    ugv_stops_dict = {}

    df_list = df.values.tolist()
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    # print(df_list)
    # for i in range(len(df)):
    if num_stops == 2:
        ugv_stops_list1 = []
        ugv_stops_list2 = []
        for j in range(len(df_list)):
            if 25 <= j <= 31:
                ugv_stops_list1.append(df_list[j])
            elif 38 <= j <= 47:
                ugv_stops_list2.append(df_list[j])
        # print(ugv_stops_list1, ugv_stops_list2)
        # print(len(ugv_stops_list1), len(ugv_stops_list2))

        gen_list = [ugv_stops_list1, ugv_stops_list2]
        ugv_Stops = [p for p in product(*gen_list)]

        for k in range(len(ugv_Stops)):
            ugv_Stops[k] = list(ugv_Stops[k])
        # print(ugv_Stops)
        # print(len(ugv_Stops))

        rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
        # for r in range(len(ugv_Stops)):
        #     rev_ugv_stops_lst.append(ugv_Stops[r])

        for i in range(len(rev_ugv_stops_lst)):
            rev_ugv_stops_lst[i].reverse()

        for l in range(len(rev_ugv_stops_lst)):
            ugv_Stops.append(rev_ugv_stops_lst[l])

        # print(ugv_Stops)

        for i in range(len(ugv_Stops)):
            ugv_stops_dict[i+2] = ugv_Stops[i]

        # print(ugv_stops_dict)
        return ugv_stops_dict
    elif num_stops == 3:
        ugv_stops_list1 = []
        ugv_stops_list2 = []
        ugv_stops_list3 = []
        for j in range(len(df_list)):
            if 25 <= j <= 31:
                ugv_stops_list1.append(df_list[j])
            elif j == 38:
                ugv_stops_list2.append(df_list[j])
            elif 39 <= j <= 47:
                ugv_stops_list3.append(df_list[j])
        # print(ugv_stops_list1, ugv_stops_list2)
        # print(len(ugv_stops_list1), len(ugv_stops_list2))

        gen_list = [ugv_stops_list1, ugv_stops_list2, ugv_stops_list3]
        ugv_Stops = [p for p in product(*gen_list)]

        for k in range(len(ugv_Stops)):
            ugv_Stops[k] = list(ugv_Stops[k])
        # print(ugv_Stops)
        # print(len(ugv_Stops))

        rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
        # for r in range(len(ugv_Stops)):
        #     rev_ugv_stops_lst.append(ugv_Stops[r])

        for i in range(len(rev_ugv_stops_lst)):
            rev_ugv_stops_lst[i].reverse()

        for l in range(len(rev_ugv_stops_lst)):
            ugv_Stops.append(rev_ugv_stops_lst[l])

        # print(ugv_Stops)
        # random.seed(123)
        # ugv_stops_shortlisted = random.sample(ugv_Stops, 254)
        for i in range(len(ugv_Stops)):
            ugv_stops_dict[i+2] = ugv_Stops[i]

        # print(ugv_stops_dict)
        return ugv_stops_dict


dictionary = scenario_1_data_processing(2)
print(dictionary)


combo = []


# def ugv_distance_calc(stop1, stop2):
#     stop1 = list(stop1)
#     stop2 = list(stop2)
#     stop1[0] = round((stop1[0]/5280), 2)
#     stop1[1] = round((stop1[1]/5280), 2)
#     stop2[0] = round((stop2[0]/5280), 2)
#     stop2[1] = round((stop2[1]/5280), 2)
#     stop1 = tuple(stop1)
#     stop2 = tuple(stop2)
#     ugv_stops_distance_dict = {}
#     df = pd.read_csv('Scenario 1 data points.csv')
#     df_list = df.values.tolist()
#     for i in range(len(df_list)):
#         df_list[i][0] = round(df_list[i][0], 2)
#         df_list[i][1] = round(df_list[i][1], 2)
#         df_list[i] = tuple(df_list[i])
#     combo = combinations(df_list, 2)
#     combo = list(combo)
#     for i in range(len(combo)):
#         combo[i] = list(combo[i])
#     rev_ugv_stops_lst = copy.deepcopy(combo)
#     # for r in range(len(ugv_Stops)):
#     #     rev_ugv_stops_lst.append(ugv_Stops[r])
#
#     for i in range(len(rev_ugv_stops_lst)):
#         rev_ugv_stops_lst[i].reverse()
#
#     for l in range(len(rev_ugv_stops_lst)):
#         combo.append(rev_ugv_stops_lst[l])
#     # print(combo)
#     for j in range(len(combo)):
#         combo[j] = tuple(combo[j])
#         ugv_stops_distance_dict[combo[j]] = ugv_distance(combo[j][0], combo[j][1])
#     # print(ugv_stops_distance_dict)
#     ugv_distance_keys = list(ugv_stops_distance_dict.keys())
#     ugv_stops_distance_dict_value = 0
#     for k in range(len(ugv_distance_keys)):
#         if stop1 == stop2:
#             ugv_stops_distance_dict_value = 0
#             break
#         if ugv_distance_keys[k][0] == stop1 and ugv_distance_keys[k][1] == stop2:
#             ugv_stops_distance_dict_value = ugv_stops_distance_dict[(ugv_distance_keys[k][0], ugv_distance_keys[k][1])]
#             break
#     return int(ugv_stops_distance_dict_value * 5280)


def ugv_distance(position_1, position_2):  # This function calculates the distance UGV travels from one mission point to another
    df = pd.read_csv('Scenario 1 data points.csv')
    df = df.round(2)
    data_pt_list = df.values.tolist()
    data_pt_list = [tuple(data_pt_list[i]) for i in range(len(data_pt_list))]
    data_pt_dict = {}
    for key in range(len(data_pt_list)):
        data_pt_dict[key] = data_pt_list[key]
    # print(data_pt_dict)
    mid_pt = (3.97, 0.49)
    stop1 = 0
    stop2 = 0
    counter1 = 0
    counter2 = 0
    ugv_Dist = 0
    list1 = []
    list2 = []
    list3 = []
    for key, val in data_pt_dict.items():
        if key <= 15:
            list1.append(val)
        elif 15 < key <= 31:
            list2.append(val)
        elif 31 < key <= 47:
            list3.append(val)
    for i in range(len(list1)):
        if (list1.count(position_1) and list1.count(position_2)) or (list2.count(position_1) and list2.count(position_2)) or (list3.count(position_1) and list3.count(position_2)):
            ugv_Dist = euclidean_distance(position_1, position_2)
        elif (list1.count(position_1) and list2.count(position_2)) or (list2.count(position_1) and list3.count(position_2)) or (list1.count(position_1) and list3.count(position_2)):
            ugv_Dist = euclidean_distance(position_1, mid_pt) + euclidean_distance(mid_pt, position_2)
        elif (list1.count(position_2) and list2.count(position_1)) or (list2.count(position_2) and list3.count(position_1)) or (list1.count(position_2) and list3.count(position_1)):
            ugv_Dist = euclidean_distance(position_2, mid_pt) + euclidean_distance(mid_pt, position_1)
    # for key, val in data_pt_dict.items():
    #     if val != position_1 and key <= 31:
    #         counter1 += 1
    #         ugv_Dist += euclidean_distance(data_pt_dict[key], data_pt_dict[key+1])
    #     else:
    #         stop1 = position_1
    #         break
    # for key, val in data_pt_dict.items():
    #     if val == position_2:
    #         stop2 = position_2
    #         break
    #     else:
    #         counter2 += 1
    #         ugv_Dist += euclidean_distance(data_pt_dict[key], data_pt_dict[key+1])
    return ugv_Dist


def euclidean_distance(position_1, position_2):
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])), 2)


# if __name__ == '__main__':
#     # ugv_dist = ugv_distance_calc((7.06, 3.02), (4.8, 3.81))
#     # print(ugv_dist)
#     lol = scenario_1_data_processing()
