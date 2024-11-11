import random
from itertools import product, combinations
import pandas as pd
import numpy as np
import ugv_power_consumption
import math
# from scenario_1_data_processing import ugv_distance_calc
import copy


def euclidean_distance(position_1, position_2):  # This function calculates the distance UAV travels from one mission point to another
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])), 2)


# def multiple_UGV_stop_locs(num_stops):
#     df = pd.read_csv('Scenario dataset/Scenario 1 data points.csv')
#     ugv_stops_dict = {}
#
#     df_list = df.values.tolist()
#     random.seed(123)
#     for i in range(len(df_list)):
#         df_list[i][0] = round(df_list[i][0], 2)
#         df_list[i][1] = round(df_list[i][1], 2)
#         df_list[i] = tuple(df_list[i])
#     # print(df_list)
#     # for i in range(len(df)):
#     # if num_stops == 2:
#     #     ugv_stops_list1 = []
#     #     ugv_stops_list2 = []
#     #     for j in range(len(df_list)):
#     #         if 5 <= j <= 11:
#     #             ugv_stops_list1.append(df_list[j])
#     #         elif 29 <= j <= 34 or j == 22 or j == 36:
#     #             ugv_stops_list2.append(df_list[j])
#     #     # print(ugv_stops_list1, ugv_stops_list2)
#     #     # print(len(ugv_stops_list1), len(ugv_stops_list2))
#     #     gen_list = [ugv_stops_list1, ugv_stops_list2]
#     #     ugv_Stops = [p for p in product(*gen_list)]
#     #
#     #     for k in range(len(ugv_Stops)):
#     #         ugv_Stops[k] = list(ugv_Stops[k])
#
#     if num_stops == 3:
#         ugv_stops_list1 = []
#         ugv_stops_list2 = []
#         ugv_stops_list3 = []
#         for j in range(len(df_list)):
#             if 25 <= j <= 31:
#                 ugv_stops_list1.append(df_list[j])
#             elif 38 <= j <= 42:
#                 ugv_stops_list2.append(df_list[j])
#             elif 43 <= j <= 47:
#                 ugv_stops_list3.append(df_list[j])
#             # print(ugv_stops_list1, ugv_stops_list2)
#             # print(len(ugv_stops_list1), len(ugv_stops_list2))
#         gen_list = [ugv_stops_list1, ugv_stops_list2, ugv_stops_list3]
#         ugv_Stops = [p for p in product(*gen_list)]
#
#         for k in range(len(ugv_Stops)):
#             ugv_Stops[k] = list(ugv_Stops[k])
#         # print(ugv_Stops)
#         # print(len(ugv_Stops))
#
#         rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
#         # for r in range(len(ugv_Stops)):
#         #     rev_ugv_stops_lst.append(ugv_Stops[r])
#
#         for i in range(len(rev_ugv_stops_lst)):
#             rev_ugv_stops_lst[i].reverse()
#
#         for l in range(len(rev_ugv_stops_lst)):
#             ugv_Stops.append(rev_ugv_stops_lst[l])
#
#         # print(ugv_Stops)
#         ugv_stops_shortlisted = random.sample(ugv_Stops, 254)
#         for j in range(254):
#             ugv_stops_dict[j+2] = ugv_stops_shortlisted[j]
#
#         return ugv_stops_dict
#     elif num_stops == 4:
#         ugv_stops_list1 = []
#         ugv_stops_list2 = []
#         ugv_stops_list3 = []
#         ugv_stops_list4 = []
#         for j in range(len(df_list)):
#             if 5 <= j <= 10:
#                 ugv_stops_list1.append(df_list[j])
#             elif j == 11 or 33 <= j <= 34:
#                 ugv_stops_list2.append(df_list[j])
#             elif 28 <= j <= 32:
#                 ugv_stops_list3.append(df_list[j])
#             elif 21 <= j <= 22 or 23 <= j <= 27:
#                 ugv_stops_list4.append(df_list[j])
#             # print(ugv_stops_list1, ugv_stops_list2)
#             # print(len(ugv_stops_list1), len(ugv_stops_list2))
#         gen_list = [ugv_stops_list1, ugv_stops_list2, ugv_stops_list3, ugv_stops_list4]
#         ugv_Stops = [p for p in product(*gen_list)]
#
#         # for k in range(len(ugv_Stops)):
#         #     ugv_Stops[k] = list(ugv_Stops[k])
#         # # print(ugv_Stops)
#         # # print(len(ugv_Stops))
#         #
#         # rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
#         # # for r in range(len(ugv_Stops)):
#         # #     rev_ugv_stops_lst.append(ugv_Stops[r])
#         #
#         # for i in range(len(rev_ugv_stops_lst)):
#         #     rev_ugv_stops_lst[i].reverse()
#         #
#         # for l in range(len(rev_ugv_stops_lst)):
#         #     ugv_Stops.append(rev_ugv_stops_lst[l])
#
#         # print(ugv_Stops)
#
#         ugv_stops_shortlisted = random.sample(ugv_Stops, 254)
#         for j in range(254):
#             ugv_stops_dict[j+2] = ugv_stops_shortlisted[j]
#
#         return ugv_stops_dict
#     else:
#         ugv_stops_list1 = []
#         ugv_stops_list2 = []
#         ugv_stops_list3 = []
#         ugv_stops_list4 = []
#         ugv_stops_list5 = []
#         for j in range(len(df_list)):
#             if 5 <= j <= 8:
#                 ugv_stops_list1.append(df_list[j])
#             elif j == 34 or 9 <= j <= 11:
#                 ugv_stops_list2.append(df_list[j])
#             elif 31 <= j <= 33:
#                 ugv_stops_list3.append(df_list[j])
#             elif 26 <= j <= 30:
#                 ugv_stops_list4.append(df_list[j])
#             elif 21 <= j <= 25:
#                 ugv_stops_list5.append(df_list[j])
#             # print(ugv_stops_list1, ugv_stops_list2)
#             # print(len(ugv_stops_list1), len(ugv_stops_list2))
#         gen_list = [ugv_stops_list1, ugv_stops_list2, ugv_stops_list3, ugv_stops_list4, ugv_stops_list5]
#         ugv_Stops = [p for p in product(*gen_list)]
#
#         # for k in range(len(ugv_Stops)):
#         #     ugv_Stops[k] = list(ugv_Stops[k])
#         # # print(ugv_Stops)
#         # # print(len(ugv_Stops))
#         #
#         # rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
#         # # for r in range(len(ugv_Stops)):
#         # #     rev_ugv_stops_lst.append(ugv_Stops[r])
#         #
#         # for i in range(len(rev_ugv_stops_lst)):
#         #     rev_ugv_stops_lst[i].reverse()
#         #
#         # for l in range(len(rev_ugv_stops_lst)):
#         #     ugv_Stops.append(rev_ugv_stops_lst[l])
#
#         # print(ugv_Stops)
#
#         ugv_stops_shortlisted = random.sample(ugv_Stops, 254)
#         for j in range(254):
#             ugv_stops_dict[j+2] = ugv_stops_shortlisted[j]
#
#         return ugv_stops_dict


def ugv_distance_calc(stop1, stop2, ugv_stops_distance_dict):
    stop1 = list(stop1)
    stop2 = list(stop2)
    stop1[0] = round((stop1[0]/5280), 2)
    stop1[1] = round((stop1[1]/5280), 2)
    stop2[0] = round((stop2[0]/5280), 2)
    stop2[1] = round((stop2[1]/5280), 2)
    stop1 = tuple(stop1)
    stop2 = tuple(stop2)
    ugv_distance_keys = list(ugv_stops_distance_dict.keys())
    ugv_stops_distance_dict_value = 0
    for k in range(len(ugv_distance_keys)):
        if stop1 == stop2:
            ugv_stops_distance_dict_value = 0
            break
        if ugv_distance_keys[k][0] == stop1 and ugv_distance_keys[k][1] == stop2:
            ugv_stops_distance_dict_value = ugv_stops_distance_dict[(ugv_distance_keys[k][0], ugv_distance_keys[k][1])]
            break
    return int(ugv_stops_distance_dict_value * 5280)


def ugv_data(strt_pt, start_loc, num_stops, ugv_stop_loc, ugv_stops_dict, NW_stop, SE_stop, mission_pts_stp1_stop, mission_pts_stp2_stop, ugv_stops_distance_dict):
    n_depot_1 = 6
    n_stop_1 = 6
    n_stop_2 = 6
    n_depot_2 = 6
    data = {}
    fuel_capacity = 287700
    n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
    ordered_mission_pts_stp1_stop = mission_pts_stp1_stop
    print(len(ordered_mission_pts_stp1_stop))

    ordered_mission_pts_stp2_stop = mission_pts_stp2_stop
    print(len(ordered_mission_pts_stp2_stop))
    stp_2_list = [(0.27, 0.03), (0.53, 0.07), (0.79, 0.1), (1.06, 0.13), (1.32, 0.16), (1.59, 0.19),
                  (1.85, 0.23), (2.12, 0.26), (2.38, 0.29), (2.65, 0.32), (2.91, 0.36), (3.18, 0.39), (3.44, 0.42), (3.71, 0.45),
                  (3.97, 0.49)]
    if num_stops == 3:
        stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
        stp1_ugv_stop_list_copy = []
        for i in range(len(stp1_ugv_stop_list)):
            stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
        stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
        stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
        stp2_ugv_stop_list_copy = []
        for i in range(len(stp2_ugv_stop_list)):
            stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
        stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
        stp3_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][2]]
        stp3_ugv_stop_list_copy = []
        for i in range(len(stp3_ugv_stop_list)):
            stp3_ugv_stop_list_copy.append(stp3_ugv_stop_list[i])
        stp3_ugv_stop = stp3_ugv_stop_list_copy.pop()
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        stp3_ugv_stp_ft_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
        stp3_ugv_stop_ft = tuple(stp3_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop, stp3_ugv_stop)
        start_loc_mi = copy.deepcopy(start_loc)
        start_loc_mi = list(start_loc_mi)
        start_loc_mi[0] = round((start_loc[0] / 5280), 2)
        start_loc_mi[1] = round((start_loc[1] / 5280), 2)
        start_loc_mi = tuple(start_loc_mi)

        _locations = ([start_loc_mi] +
                      [start_loc_mi] * n_depot_1 +
                      stp1_ugv_stop_list * (n_stop_1 - 2) +
                      stp2_ugv_stop_list * (n_stop_2 - 2) +
                      stp3_ugv_stop_list * (n_stop_2 - 2) +
                      [(0, 0)] * (n_depot_2 - 1) +
                      [start_loc_mi]
                      )
        mission_locations = []
        count = 0
        # count_se = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            count += 1
            if ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            elif ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            else:
                continue

        for i in range(len(stp_2_list)):
            mission_locations.append(stp_2_list[i])

        for i in range(len(mission_locations)):
            _locations.append(mission_locations[i])

        ugv_vel = 13
        depot_to_first_stop_vel = 13
        total_mission_pts = len(mission_locations)
        data["total_mission_pts"] = len(_locations)
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        stp1_wait_time = int(NW_stop) * 60
        stp2_wait_time = int(NW_stop) * 60
        stp3_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft, ugv_stops_distance_dict)
        dist2 = ugv_distance_calc(stp2_ugv_stop_ft, stp3_ugv_stop_ft, ugv_stops_distance_dict)
        if strt_pt == 2:
            depot_b_dist = ugv_distance_calc(start_loc, stp1_ugv_stop_ft, ugv_stops_distance_dict)
        else:
            depot_b_dist = ugv_distance_calc(start_loc, stp1_ugv_stop_ft, ugv_stops_distance_dict)
        ugv_travel_time = (dist // int(ugv_vel)) + 926
        velocity = int(ugv_vel)
        ugv_travel_time_2 = (dist2 // int(ugv_vel)) + 926
        stp1_stop_tw_1 = depot_b_dist // int(depot_to_first_stop_vel)  # this tells the time at which UGV arrives NW/SE stop from depot
        depot_b_to_nw_velocity = int(depot_to_first_stop_vel)
        stp1_stop_tw_2 = stp1_stop_tw_1 + stp1_wait_time
        stp2_stop_tw_1 = stp1_stop_tw_2 + ugv_travel_time
        stp2_stop_tw_2 = stp1_stop_tw_2 + ugv_travel_time + stp2_wait_time
        stp3_stop_tw_1 = stp2_stop_tw_2 + ugv_travel_time_2
        stp3_stop_tw_2 = stp3_stop_tw_1 + stp3_wait_time
        print(depot_b_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(stp1_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        if strt_pt == 2:
            return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
            return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), (3.97*5280, 0.49*5280), ugv_stops_distance_dict)
            return_dist_to_depotB = ugv_distance_calc((3.97*5280, 0.49*5280), start_loc, ugv_stops_distance_dict)
            return_time_1 = (return_dist_from_se//velocity)
            return_time_2 = (return_dist_to_depotB//depot_b_to_nw_velocity)
            veh_max_time = stp3_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotB//depot_b_to_nw_velocity)
        elif strt_pt == 3:
            return_dist_from_se = 0
            return_dist_to_depotA = 0
            for i in range(len(ordered_mission_pts_stp1_stop)):
                if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                    return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
                    return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp3_ugv_stop_ft, ugv_stops_distance_dict)
                    if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp3_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    break
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
                    return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft, ugv_stops_distance_dict)
                    if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp3_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    break
            return_time_1 = (return_dist_from_se//velocity)
            return_time_2 = (return_dist_to_depotA//velocity)
            veh_max_time = stp3_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
        elif strt_pt == 4:
            return_dist_from_se = 0
            return_dist_to_depotA = 0
            for i in range(len(ordered_mission_pts_stp1_stop)):
                if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                    return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
                    return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp3_ugv_stop_ft, ugv_stops_distance_dict)
                    if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp3_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    break
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
                    return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft, ugv_stops_distance_dict)
                    if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp3_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                        return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    break
            return_time_1 = (return_dist_from_se//velocity)
            return_time_2 = (return_dist_to_depotA//velocity)
            veh_max_time = stp3_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
        else:
            return_dist_from_se = 0
            return_dist_to_depotC = 0
            for i in range(len(ordered_mission_pts_stp1_stop)):
                if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                    return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
                    return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft, ugv_stops_distance_dict)
                    if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                        return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                        return_dist_to_depotC = ugv_distance_calc(stp3_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    break
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_from_se_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
                    return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp3_ugv_stop_ft, ugv_stops_distance_dict)
                    if ordered_mission_pts_stp1_stop[i] == stp3_ugv_stop:
                        return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                        return_dist_to_depotC = ugv_distance_calc(stp3_ugv_stop_ft, start_loc, ugv_stops_distance_dict)
                    break
            return_time_1 = (return_dist_from_se//velocity)
            return_time_2 = (return_dist_to_depotC//velocity)
            veh_max_time = stp3_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
        ugv_time_windows = ([(0, 60)] +  # 0 (start)
                            [(0, veh_max_time)] * n_depot_1 +  # 1-6
                            [(stp1_stop_tw_1, stp1_stop_tw_2)] * (n_stop_1 - 2) +  # 7-10
                            [(stp2_stop_tw_1, stp2_stop_tw_2)] * (n_stop_2 - 2) +  # 11-14
                            [(stp3_stop_tw_1, stp3_stop_tw_2)] * (n_stop_2 - 2) +  # 15-18
                            [(0, veh_max_time)] * n_depot_2 +  # 19-24
                            [(0, veh_max_time)] * total_mission_pts
                            )

        total_ugv_power = ugv_power_consumption.ugv_power(stp1_wait_time, stp3_wait_time, velocity//3.281, depot_b_to_nw_velocity//3.281, stp1_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
        if total_ugv_power > 25010000:
            print("******************************************************")
            print("UGV route goes infeasible although UAV has a solution")
            print("******************************************************")
            pen = 5000000
        else:
            pen = 0
        print(total_ugv_power)

        data['time_windows'] = ugv_time_windows

        data['counter'] = ([0] +  # 0 (start)
                           [0] * (n_sum_rch_stops - 1) +  # 1-23 (stations)
                           [0] +  # 24 (end)
                           [1] * total_mission_pts)
        # print(len(data["counter"]))
        # print(sum(data["counter"]))
        data["num_vehicles"] = 1
        data["fuel_capacity"] = fuel_capacity
        data["horizon"] = veh_max_time
        data["vehicle_speed"] = 33
        data["starts"] = [0]
        data["ends"] = [n_sum_rch_stops]
        print('End node is {}'.format(data["ends"]))
        data["n_sum_rch_stops"] = n_sum_rch_stops
        distance_matrix = np.zeros((data["num_locations"], data["num_locations"]), dtype=int)
        for i in range(data["num_locations"]):
            for j in range(data["num_locations"]):
                if i == j:
                    distance_matrix[i][j] = 0
                else:
                    distance_matrix[i][j] = euclidean_distance(data["locations"][i], data["locations"][j])
        dist_matrix = distance_matrix.tolist()
        data["distance_matrix"] = dist_matrix
        assert len(data['distance_matrix']) == len(data['locations'])
        assert len(data['distance_matrix']) == len(data['time_windows'])
        assert len(data['starts']) == len(data['ends'])
        assert data['num_vehicles'] == len(data['starts'])
        assert len(data["counter"]) == len(data['time_windows'])
        data["stations"] = [i for i in range(0, n_sum_rch_stops)]
        data["visits"] = [i for i in range(25, len(_locations))]
        print(data["stations"])
        print(data["visits"])
        print(ugv_time_windows)
        return data, veh_max_time, depot_b_to_nw_velocity, velocity, pen
    # elif num_stops == 4:
    #     stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
    #     stp1_ugv_stop_list_copy = []
    #     for i in range(len(stp1_ugv_stop_list)):
    #         stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
    #     stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
    #     stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
    #     stp2_ugv_stop_list_copy = []
    #     for i in range(len(stp2_ugv_stop_list)):
    #         stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
    #     stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
    #     stp3_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][2]]
    #     stp3_ugv_stop_list_copy = []
    #     for i in range(len(stp3_ugv_stop_list)):
    #         stp3_ugv_stop_list_copy.append(stp3_ugv_stop_list[i])
    #     stp3_ugv_stop = stp3_ugv_stop_list_copy.pop()
    #     stp4_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][3]]
    #     stp4_ugv_stop_list_copy = []
    #     for i in range(len(stp4_ugv_stop_list)):
    #         stp4_ugv_stop_list_copy.append(stp4_ugv_stop_list[i])
    #     stp4_ugv_stop = stp4_ugv_stop_list_copy.pop()
    #     stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
    #     stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
    #     stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
    #     stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
    #     stp3_ugv_stp_ft_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
    #     stp3_ugv_stop_ft = tuple(stp3_ugv_stp_ft_list)
    #     stp4_ugv_stp_ft_list = [stp4_ugv_stop[i]*5280 for i in range(len(stp4_ugv_stop))]
    #     stp4_ugv_stop_ft = tuple(stp4_ugv_stp_ft_list)
    #     print(stp1_ugv_stop, stp2_ugv_stop, stp3_ugv_stop, stp4_ugv_stop)
    #
    #     _locations = ([start_loc] +
    #                   [start_loc] * n_depot_1 +
    #                   stp1_ugv_stop_list * (n_stop_1 - 3) +
    #                   stp2_ugv_stop_list * (n_stop_2 - 3) +
    #                   stp3_ugv_stop_list * (n_stop_2 - 3) +
    #                   stp4_ugv_stop_list * (n_stop_2 - 3) +
    #                   [(0.6, 8.07)] * (n_depot_2 - 1) +
    #                   [start_loc]
    #                   )
    #     mission_locations = []
    #     count = 0
    #     count_se = 0
    #     for i in range(len(ordered_mission_pts_stp1_stop)):
    #         count += 1
    #         if ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #             for j in range(count-1):
    #                 mission_locations.append(ordered_mission_pts_stp1_stop[j])
    #             break
    #         elif ordered_mission_pts_stp1_stop[i] == stp4_ugv_stop:
    #             for j in range(count-1):
    #                 mission_locations.append(ordered_mission_pts_stp1_stop[j])
    #             break
    #         else:
    #             continue
    #     for i in range(len(ordered_mission_pts_stp2_stop)):
    #         count_se += 1
    #         if ordered_mission_pts_stp2_stop[i] == stp4_ugv_stop:
    #             for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
    #                 mission_locations.append(ordered_mission_pts_stp2_stop[j])
    #             break
    #         elif ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
    #             for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
    #                 mission_locations.append(ordered_mission_pts_stp2_stop[j])
    #             break
    #         else:
    #             continue
    #     for i in range(len(mission_locations)):
    #         _locations.append(mission_locations[i])
    #
    #     ugv_vel = 15
    #     depot_to_first_stop_vel = 15
    #     total_mission_pts = len(mission_locations)
    #     data["total_mission_pts"] = len(_locations)
    #     data["coordinates"] = _locations
    #     data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
    #     data["num_locations"] = len(data["locations"])
    #     stp1_wait_time = int(NW_stop) * 60
    #     stp2_wait_time = int(NW_stop) * 60
    #     stp3_wait_time = int(NW_stop) * 60
    #     stp4_wait_time = int(SE_stop) * 60
    #     dist = ugv_distance_calc_auto.distance(stp1_ugv_stop_ft, stp2_ugv_stop_ft)
    #     dist2 = ugv_distance_calc_auto.distance(stp2_ugv_stop_ft, stp3_ugv_stop_ft)
    #     dist3 = ugv_distance_calc_auto.distance(stp3_ugv_stop_ft, stp4_ugv_stop_ft)
    #     if strt_pt == 2:
    #         depot_b_dist = ugv_distance_calc_auto.distance_from_depotB(start_loc, stp1_ugv_stop_ft)
    #     else:
    #         depot_b_dist = ugv_distance_calc_auto.distance(start_loc, stp1_ugv_stop_ft)
    #     ugv_travel_time = (dist // int(ugv_vel)) + 926
    #     velocity = int(ugv_vel)
    #     ugv_travel_time_2 = (dist2 // int(ugv_vel)) + 926
    #     ugv_travel_time_3 = (dist3 // int(ugv_vel)) + 926
    #     stp1_stop_tw_1 = depot_b_dist // int(depot_to_first_stop_vel)  # this tells the time at which UGV arrives NW/SE stop from depot
    #     depot_b_to_nw_velocity = int(depot_to_first_stop_vel)
    #     stp1_stop_tw_2 = stp1_stop_tw_1 + stp1_wait_time
    #     stp2_stop_tw_1 = stp1_stop_tw_2 + ugv_travel_time
    #     stp2_stop_tw_2 = stp1_stop_tw_2 + ugv_travel_time + stp2_wait_time
    #     stp3_stop_tw_1 = stp2_stop_tw_2 + ugv_travel_time_2
    #     stp3_stop_tw_2 = stp3_stop_tw_1 + stp3_wait_time
    #     stp4_stop_tw_1 = stp3_stop_tw_2 + ugv_travel_time_3
    #     stp4_stop_tw_2 = stp4_stop_tw_1 + stp4_wait_time
    #     print(depot_b_to_nw_velocity, velocity)
    #     print("The travel time for UGV from depot B to NW UGV stop is: {}".format(stp1_stop_tw_1))
    #     print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
    #     if strt_pt == 2:
    #         return_dist_from_se_list = [stp4_ugv_stop[i]*5280 for i in range(len(stp4_ugv_stop))]
    #         return_dist_from_se = ugv_distance_calc_auto.distance(tuple(return_dist_from_se_list), (3.79*5280, 6.71*5280))
    #         return_dist_to_depotB = ugv_distance_calc_auto.distance_from_depotB((3.79*5280, 6.71*5280), start_loc)
    #         return_time_1 = (return_dist_from_se//velocity)
    #         return_time_2 = (return_dist_to_depotB//depot_b_to_nw_velocity)
    #         veh_max_time = stp4_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotB//depot_b_to_nw_velocity)
    #     else:
    #         return_dist_from_se = 0
    #         return_dist_to_depotA = 0
    #         for i in range(len(ordered_mission_pts_stp1_stop)):
    #             if ordered_mission_pts_stp1_stop[i] == stp4_ugv_stop:
    #                 return_dist_from_se_list = [stp4_ugv_stop[i]*5280 for i in range(len(stp4_ugv_stop))]
    #                 return_dist_from_se = ugv_distance_calc_auto.distance(tuple(return_dist_from_se_list), stp4_ugv_stop_ft)
    #                 if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp4_ugv_stop_ft, start_loc)
    #                 elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp1_ugv_stop_ft, start_loc)
    #                 break
    #             elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #                 return_dist_from_se_list = [stp4_ugv_stop[i]*5280 for i in range(len(stp4_ugv_stop))]
    #                 return_dist_from_se = ugv_distance_calc_auto.distance(tuple(return_dist_from_se_list), stp1_ugv_stop_ft)
    #                 if ordered_mission_pts_stp1_stop[i] == stp4_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp4_ugv_stop_ft, start_loc)
    #                 elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp1_ugv_stop_ft, start_loc)
    #                 break
    #         return_time_1 = (return_dist_from_se//velocity)
    #         return_time_2 = (return_dist_to_depotA//velocity)
    #         veh_max_time = stp4_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
    #     ugv_time_windows = ([(0, 60)] +  # 0 (start)
    #                         [(0, veh_max_time)] * n_depot_1 +  # 1-6
    #                         [(stp1_stop_tw_1, stp1_stop_tw_2)] * (n_stop_1 - 3) +  # 7-9
    #                         [(stp2_stop_tw_1, stp2_stop_tw_2)] * (n_stop_2 - 3) +  # 10-12
    #                         [(stp3_stop_tw_1, stp3_stop_tw_2)] * (n_stop_2 - 3) +  # 13-15
    #                         [(stp4_stop_tw_1, stp4_stop_tw_2)] * (n_stop_2 - 3) +  # 16-18
    #                         [(0, veh_max_time)] * n_depot_2 +  # 19-24
    #                         [(0, veh_max_time)] * total_mission_pts
    #                         )
    #
    #     total_ugv_power = ugv_power_consumption.ugv_power(stp1_wait_time, stp2_wait_time, velocity//3.281, depot_b_to_nw_velocity//3.281, stp1_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
    #     if total_ugv_power > 25010000:
    #         print("******************************************************")
    #         print("UGV route goes infeasible although UAV has a solution")
    #         print("******************************************************")
    #         pen = 5000000
    #     else:
    #         pen = 0
    #     print(total_ugv_power)
    #
    #     data['time_windows'] = ugv_time_windows
    #
    #     data['counter'] = ([0] +  # 0 (start)
    #                        [0] * (n_sum_rch_stops - 1) +  # 1-23 (stations)
    #                        [0] +  # 24 (end)
    #                        [1] * total_mission_pts)
    #     # print(len(data["counter"]))
    #     # print(sum(data["counter"]))
    #     data["num_vehicles"] = 1
    #     data["fuel_capacity"] = fuel_capacity
    #     data["horizon"] = veh_max_time
    #     data["vehicle_speed"] = 33
    #     data["starts"] = [0]
    #     data["ends"] = [n_sum_rch_stops]
    #     print('End node is {}'.format(data["ends"]))
    #     data["n_sum_rch_stops"] = n_sum_rch_stops
    #     distance_matrix = np.zeros((data["num_locations"], data["num_locations"]), dtype=int)
    #     for i in range(data["num_locations"]):
    #         for j in range(data["num_locations"]):
    #             if i == j:
    #                 distance_matrix[i][j] = 0
    #             else:
    #                 distance_matrix[i][j] = euclidean_distance(data["locations"][i], data["locations"][j])
    #     dist_matrix = distance_matrix.tolist()
    #     data["distance_matrix"] = dist_matrix
    #     assert len(data['distance_matrix']) == len(data['locations'])
    #     assert len(data['distance_matrix']) == len(data['time_windows'])
    #     assert len(data['starts']) == len(data['ends'])
    #     assert data['num_vehicles'] == len(data['starts'])
    #     assert len(data["counter"]) == len(data['time_windows'])
    #     data["stations"] = [i for i in range(0, n_sum_rch_stops)]
    #     data["visits"] = [i for i in range(25, len(_locations))]
    #     print(data["stations"])
    #     print(data["visits"])
    #     return data, veh_max_time, depot_b_to_nw_velocity, velocity, pen
    # else:
    #     stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
    #     stp1_ugv_stop_list_copy = []
    #     for i in range(len(stp1_ugv_stop_list)):
    #         stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
    #     stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
    #     stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
    #     stp2_ugv_stop_list_copy = []
    #     for i in range(len(stp2_ugv_stop_list)):
    #         stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
    #     stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
    #     stp3_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][2]]
    #     stp3_ugv_stop_list_copy = []
    #     for i in range(len(stp3_ugv_stop_list)):
    #         stp3_ugv_stop_list_copy.append(stp3_ugv_stop_list[i])
    #     stp3_ugv_stop = stp3_ugv_stop_list_copy.pop()
    #     stp4_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][3]]
    #     stp4_ugv_stop_list_copy = []
    #     for i in range(len(stp4_ugv_stop_list)):
    #         stp4_ugv_stop_list_copy.append(stp4_ugv_stop_list[i])
    #     stp4_ugv_stop = stp4_ugv_stop_list_copy.pop()
    #     stp5_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][4]]
    #     stp5_ugv_stop_list_copy = []
    #     for i in range(len(stp5_ugv_stop_list)):
    #         stp5_ugv_stop_list_copy.append(stp5_ugv_stop_list[i])
    #     stp5_ugv_stop = stp5_ugv_stop_list_copy.pop()
    #     stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
    #     stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
    #     stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
    #     stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
    #     stp3_ugv_stp_ft_list = [stp3_ugv_stop[i]*5280 for i in range(len(stp3_ugv_stop))]
    #     stp3_ugv_stop_ft = tuple(stp3_ugv_stp_ft_list)
    #     stp4_ugv_stp_ft_list = [stp4_ugv_stop[i]*5280 for i in range(len(stp4_ugv_stop))]
    #     stp4_ugv_stop_ft = tuple(stp4_ugv_stp_ft_list)
    #     stp5_ugv_stp_ft_list = [stp5_ugv_stop[i]*5280 for i in range(len(stp5_ugv_stop))]
    #     stp5_ugv_stop_ft = tuple(stp5_ugv_stp_ft_list)
    #     print(stp1_ugv_stop, stp2_ugv_stop, stp3_ugv_stop, stp4_ugv_stop, stp5_ugv_stop)
    #
    #     _locations = ([start_loc] +
    #                   [start_loc] * n_depot_1 +
    #                   stp1_ugv_stop_list * (n_stop_1 - 4) +
    #                   stp2_ugv_stop_list * (n_stop_2 - 4) +
    #                   stp3_ugv_stop_list * (n_stop_2 - 4) +
    #                   stp4_ugv_stop_list * (n_stop_2 - 3) +
    #                   stp5_ugv_stop_list * (n_stop_2 - 3) +
    #                   [(0.6, 8.07)] * (n_depot_2 - 1) +
    #                   [start_loc]
    #                   )
    #     mission_locations = []
    #     count = 0
    #     count_se = 0
    #     for i in range(len(ordered_mission_pts_stp1_stop)):
    #         count += 1
    #         if ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #             for j in range(count-1):
    #                 mission_locations.append(ordered_mission_pts_stp1_stop[j])
    #             break
    #         elif ordered_mission_pts_stp1_stop[i] == stp5_ugv_stop:
    #             for j in range(count-1):
    #                 mission_locations.append(ordered_mission_pts_stp1_stop[j])
    #             break
    #         else:
    #             continue
    #     for i in range(len(ordered_mission_pts_stp2_stop)):
    #         count_se += 1
    #         if ordered_mission_pts_stp2_stop[i] == stp5_ugv_stop:
    #             for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
    #                 mission_locations.append(ordered_mission_pts_stp2_stop[j])
    #             break
    #         elif ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
    #             for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
    #                 mission_locations.append(ordered_mission_pts_stp2_stop[j])
    #             break
    #         else:
    #             continue
    #     for i in range(len(mission_locations)):
    #         _locations.append(mission_locations[i])
    #
    #     ugv_vel = 15
    #     depot_to_first_stop_vel = 15
    #     total_mission_pts = len(mission_locations)
    #     data["total_mission_pts"] = len(_locations)
    #     data["coordinates"] = _locations
    #     data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
    #     data["num_locations"] = len(data["locations"])
    #     stp1_wait_time = int(NW_stop) * 60
    #     stp2_wait_time = int(NW_stop) * 60
    #     stp3_wait_time = int(NW_stop) * 60
    #     stp4_wait_time = int(NW_stop) * 60
    #     stp5_wait_time = int(SE_stop) * 60
    #     dist = ugv_distance_calc_auto.distance(stp1_ugv_stop_ft, stp2_ugv_stop_ft)
    #     dist2 = ugv_distance_calc_auto.distance(stp2_ugv_stop_ft, stp3_ugv_stop_ft)
    #     dist3 = ugv_distance_calc_auto.distance(stp3_ugv_stop_ft, stp4_ugv_stop_ft)
    #     dist4 = ugv_distance_calc_auto.distance(stp4_ugv_stop_ft, stp5_ugv_stop_ft)
    #     if strt_pt == 2:
    #         depot_b_dist = ugv_distance_calc_auto.distance_from_depotB(start_loc, stp1_ugv_stop_ft)
    #     else:
    #         depot_b_dist = ugv_distance_calc_auto.distance(start_loc, stp1_ugv_stop_ft)
    #     ugv_travel_time = (dist // int(ugv_vel)) + 926
    #     velocity = int(ugv_vel)
    #     ugv_travel_time_2 = (dist2 // int(ugv_vel)) + 926
    #     ugv_travel_time_3 = (dist3 // int(ugv_vel)) + 926
    #     ugv_travel_time_4 = (dist4 // int(ugv_vel)) + 926
    #     stp1_stop_tw_1 = depot_b_dist // int(depot_to_first_stop_vel)  # this tells the time at which UGV arrives NW/SE stop from depot
    #     depot_b_to_nw_velocity = int(depot_to_first_stop_vel)
    #     stp1_stop_tw_2 = stp1_stop_tw_1 + stp1_wait_time
    #     stp2_stop_tw_1 = stp1_stop_tw_2 + ugv_travel_time
    #     stp2_stop_tw_2 = stp1_stop_tw_2 + ugv_travel_time + stp2_wait_time
    #     stp3_stop_tw_1 = stp2_stop_tw_2 + ugv_travel_time_2
    #     stp3_stop_tw_2 = stp3_stop_tw_1 + stp3_wait_time
    #     stp4_stop_tw_1 = stp3_stop_tw_2 + ugv_travel_time_3
    #     stp4_stop_tw_2 = stp4_stop_tw_1 + stp4_wait_time
    #     stp5_stop_tw_1 = stp4_stop_tw_2 + ugv_travel_time_4
    #     stp5_stop_tw_2 = stp5_stop_tw_1 + stp5_wait_time
    #     print(depot_b_to_nw_velocity, velocity)
    #     print("The travel time for UGV from depot B to NW UGV stop is: {}".format(stp1_stop_tw_1))
    #     print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
    #     if strt_pt == 2:
    #         return_dist_from_se_list = [stp5_ugv_stop[i]*5280 for i in range(len(stp5_ugv_stop))]
    #         return_dist_from_se = ugv_distance_calc_auto.distance(tuple(return_dist_from_se_list), (3.79*5280, 6.71*5280))
    #         return_dist_to_depotB = ugv_distance_calc_auto.distance_from_depotB((3.79*5280, 6.71*5280), start_loc)
    #         return_time_1 = (return_dist_from_se//velocity)
    #         return_time_2 = (return_dist_to_depotB//depot_b_to_nw_velocity)
    #         veh_max_time = stp5_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotB//depot_b_to_nw_velocity)
    #     else:
    #         return_dist_from_se = 0
    #         return_dist_to_depotA = 0
    #         for i in range(len(ordered_mission_pts_stp1_stop)):
    #             if ordered_mission_pts_stp1_stop[i] == stp5_ugv_stop:
    #                 return_dist_from_se_list = [stp5_ugv_stop[i]*5280 for i in range(len(stp5_ugv_stop))]
    #                 return_dist_from_se = ugv_distance_calc_auto.distance(tuple(return_dist_from_se_list), stp5_ugv_stop_ft)
    #                 if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp5_ugv_stop_ft, start_loc)
    #                 elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp1_ugv_stop_ft, start_loc)
    #                 break
    #             elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #                 return_dist_from_se_list = [stp5_ugv_stop[i]*5280 for i in range(len(stp5_ugv_stop))]
    #                 return_dist_from_se = ugv_distance_calc_auto.distance(tuple(return_dist_from_se_list), stp1_ugv_stop_ft)
    #                 if ordered_mission_pts_stp1_stop[i] == stp5_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp5_ugv_stop_ft, start_loc)
    #                 elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
    #                     return_dist_to_depotA = ugv_distance_calc_auto.distance(stp1_ugv_stop_ft, start_loc)
    #                 break
    #         return_time_1 = (return_dist_from_se//velocity)
    #         return_time_2 = (return_dist_to_depotA//velocity)
    #         veh_max_time = stp5_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
    #     ugv_time_windows = ([(0, 60)] +  # 0 (start)
    #                         [(0, veh_max_time)] * n_depot_1 +  # 1-6
    #                         [(stp1_stop_tw_1, stp1_stop_tw_2)] * (n_stop_1 - 4) +  # 7-8
    #                         [(stp2_stop_tw_1, stp2_stop_tw_2)] * (n_stop_2 - 4) +  # 9-10
    #                         [(stp3_stop_tw_1, stp3_stop_tw_2)] * (n_stop_2 - 4) +  # 11-12
    #                         [(stp4_stop_tw_1, stp4_stop_tw_2)] * (n_stop_2 - 3) +  # 13-15
    #                         [(stp5_stop_tw_1, stp5_stop_tw_2)] * (n_stop_2 - 3) +  # 16-18
    #                         [(0, veh_max_time)] * n_depot_2 +  # 19-24
    #                         [(0, veh_max_time)] * total_mission_pts
    #                         )
    #
    #     total_ugv_power = ugv_power_consumption.ugv_power(stp1_wait_time, stp2_wait_time, velocity//3.281, depot_b_to_nw_velocity//3.281, stp1_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
    #     if total_ugv_power > 25010000:
    #         print("******************************************************")
    #         print("UGV route goes infeasible although UAV has a solution")
    #         print("******************************************************")
    #         pen = 5000000
    #     else:
    #         pen = 0
    #     print(total_ugv_power)
    #
    #     data['time_windows'] = ugv_time_windows
    #
    #     data['counter'] = ([0] +  # 0 (start)
    #                        [0] * (n_sum_rch_stops - 1) +  # 1-23 (stations)
    #                        [0] +  # 24 (end)
    #                        [1] * total_mission_pts)
    #     # print(len(data["counter"]))
    #     # print(sum(data["counter"]))
    #     data["num_vehicles"] = 1
    #     data["fuel_capacity"] = fuel_capacity
    #     data["horizon"] = veh_max_time
    #     data["vehicle_speed"] = 33
    #     data["starts"] = [0]
    #     data["ends"] = [n_sum_rch_stops]
    #     print('End node is {}'.format(data["ends"]))
    #     data["n_sum_rch_stops"] = n_sum_rch_stops
    #     distance_matrix = np.zeros((data["num_locations"], data["num_locations"]), dtype=int)
    #     for i in range(data["num_locations"]):
    #         for j in range(data["num_locations"]):
    #             if i == j:
    #                 distance_matrix[i][j] = 0
    #             else:
    #                 distance_matrix[i][j] = euclidean_distance(data["locations"][i], data["locations"][j])
    #     dist_matrix = distance_matrix.tolist()
    #     data["distance_matrix"] = dist_matrix
    #     assert len(data['distance_matrix']) == len(data['locations'])
    #     assert len(data['distance_matrix']) == len(data['time_windows'])
    #     assert len(data['starts']) == len(data['ends'])
    #     assert data['num_vehicles'] == len(data['starts'])
    #     assert len(data["counter"]) == len(data['time_windows'])
    #     data["stations"] = [i for i in range(0, n_sum_rch_stops)]
    #     data["visits"] = [i for i in range(25, len(_locations))]
    #     print(data["stations"])
    #     print(data["visits"])
    #     return data, veh_max_time, depot_b_to_nw_velocity, velocity, pen


# if __name__ == "__main__":
#     ugv_stops = multiple_UGV_stop_locs(2)
#     ugv_stops1 = multiple_UGV_stop_locs(3)
#     ugv_stops2 = multiple_UGV_stop_locs(4)
#     ugv_stops3 = multiple_UGV_stop_locs(5)
#     print(len(ugv_stops), len(ugv_stops1), len(ugv_stops2), len(ugv_stops3))
