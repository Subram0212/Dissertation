"""UGV travels at a speed of 4.3-4.5 m/s. UAV travels at a speed of 10 m/s.
Functions applied in this inner loop UAV optimization code are described as follows:

create_data_model: create a dictionary consisting of UAV and UGV parameters required for optimization.
ugv_distance_calc: This function returns the distance needed to be traveled by UGV between any two locations.
euclidean_distance: This function calculates the distance needed to be traveld by UAV between two mission points.
print_solution: This function prints the output of the optimal UAV and UGV routes.
main: This function carries out the optimization of UAV routes for a certain UGV route using Local search and guided local search."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from time import time
import random
import ugv_power_consumption
import csv
import copy
import scenario_2_data_processing_2stops
from itertools import combinations


def create_data_model(ugv_stop_loc, NW_stop, SE_stop, strt_pt, ugv_stops_distance_dict):
    n_depot_1 = 6
    n_stop_1 = 6
    n_stop_2 = 6
    n_depot_2 = 6
    data = {}
    fuel_capacity = 287700
    ugv_vel = 10
    depot_to_first_stop_vel = 10
    # strt_pt = 5
    if strt_pt == 2:
        print("Start point is: {}".format(strt_pt))
        n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(5.35, 6.1), (5.35, 6.26), (5.34, 6.42),
                                         (5.34, 6.58), (5.34, 6.74), (5.33, 6.9), (5.33, 7.05), (5.33, 7.21), (5.32, 7.37), (5.32, 7.53), (5.32, 7.69),
                                         (5.31, 7.85), (5.31, 8.01), (5.31, 8.16), (5.3, 8.32), (5.3, 8.48)]
        ordered_mission_pts_stp1_stop = [ele for ele in reversed(ordered_mission_pts_stp1_stop)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(5.56, 5.89), (5.77, 5.83), (5.97, 5.77), (6.18, 5.71), (6.39, 5.65), (6.59, 5.59),
                                         (6.8, 5.53), (7.01, 5.47), (7.21, 5.41), (7.42, 5.35), (7.63, 5.3), (7.83, 5.24), (8.04, 5.18),
                                         (8.25, 5.12), (8.45, 5.06), (8.66, 5)]
        print(len(ordered_mission_pts_stp2_stop))
        start_loc = (0*5280, 0*5280)
        ugv_stops_dict = scenario_2_data_processing_2stops.scenario_1_data_processing()
        # ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)],
        #              [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)],
        #              [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)],
        #              [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (10.54, 0.9)],
        #              [(3.74, 10.45), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.54, 10.04), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)]]
        #
        # rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
        #
        # for i in range(len(rev_ugv_stops_lst)):
        #     rev_ugv_stops_lst[i].reverse()
        #
        # for l in range(len(rev_ugv_stops_lst)):
        #     ugv_Stops.append(rev_ugv_stops_lst[l])
        #
        # for i in range(len(ugv_Stops)):
        #     ugv_stops_dict[i+2] = ugv_Stops[i]

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
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(0, 0)] +
                      [(0, 0)] * n_depot_1 +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0, 0)] * n_depot_2
                      )
        mission_locations = []
        count = 0
        count_se = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            count += 1
            if ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            elif ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            else:
                continue
        for i in range(len(ordered_mission_pts_stp2_stop)):
            count_se += 1
            if ordered_mission_pts_stp2_stop[i] == stp2_ugv_stop:
                for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
                    mission_locations.append(ordered_mission_pts_stp2_stop[j])
                break
            elif ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
                for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
                    mission_locations.append(ordered_mission_pts_stp2_stop[j])
                break
            else:
                continue
        for i in range(len(mission_locations)):
            _locations.append(mission_locations[i])

        total_mission_pts = len(mission_locations)
        data["total_mission_pts"] = len(_locations)
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft, ugv_stops_distance_dict)
        depot_b_dist = ugv_distance_calc((0.0*5280, 0.0*5280), stp1_ugv_stop_ft, ugv_stops_distance_dict)
        ugv_travel_time = (dist // int(ugv_vel)) + 926
        velocity = int(ugv_vel)
        nw_stop_tw_1 = depot_b_dist // int(depot_to_first_stop_vel)  # this tells the time at which UGV arrives NW/SE stop from depot
        depot_b_to_nw_velocity = int(depot_to_first_stop_vel)
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        print(depot_b_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), (5.35*5280, 5.95*5280), ugv_stops_distance_dict)
        return_dist_to_depotB = ugv_distance_calc((5.35*5280, 5.95*5280), (0.0*5280, 0.0*5280), ugv_stops_distance_dict)
        return_time_1 = (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotB//depot_b_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotB//depot_b_to_nw_velocity)
        ugv_time_windows = ([(0, 60)] +  # 0 (start)
                            [(0, veh_max_time)] * n_depot_1 +  # 1-6
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 7-12
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 13-18
                            [(0, veh_max_time)] * n_depot_2 +  # 19-24
                            [(0, veh_max_time)] * total_mission_pts
                            )
        print(ugv_time_windows)

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity/3.281, depot_b_to_nw_velocity/3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, depot_b_to_nw_velocity, velocity, pen, time_windows_dict
    elif strt_pt == 3:
        print("Start point is: {}".format(strt_pt))
        n_sum_rch_stops = n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(5.35, 6.1), (5.35, 6.26), (5.34, 6.42),
                                         (5.34, 6.58), (5.34, 6.74), (5.33, 6.9), (5.33, 7.05), (5.33, 7.21), (5.32, 7.37), (5.32, 7.53), (5.32, 7.69),
                                         (5.31, 7.85), (5.31, 8.01), (5.31, 8.16), (5.3, 8.32), (5.3, 8.48)]
        ordered_mission_pts_stp1_stop = [ele for ele in reversed(ordered_mission_pts_stp1_stop)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(5.56, 5.89), (5.77, 5.83), (5.97, 5.77), (6.18, 5.71), (6.39, 5.65), (6.59, 5.59),
                                         (6.8, 5.53), (7.01, 5.47), (7.21, 5.41), (7.42, 5.35), (7.63, 5.3), (7.83, 5.24), (8.04, 5.18),
                                         (8.25, 5.12), (8.45, 5.06), (8.66, 5)]
        print(len(ordered_mission_pts_stp2_stop))

        ugv_stops_dict = scenario_2_data_processing_2stops.scenario_1_data_processing()

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
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(5.35, 5.95)] +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.0, 0.0)] * (n_depot_2)
                      # [(5.35, 5.95)]
                      )
        mission_locations = []
        count = 0
        count_se = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            count += 1
            if ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            elif ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            else:
                continue
        for i in range(len(ordered_mission_pts_stp2_stop)):
            count_se += 1
            if ordered_mission_pts_stp2_stop[i] == stp2_ugv_stop:
                for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
                    mission_locations.append(ordered_mission_pts_stp2_stop[j])
                break
            elif ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
                for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
                    mission_locations.append(ordered_mission_pts_stp2_stop[j])
                break
            else:
                continue
        for i in range(len(mission_locations)):
            _locations.append(mission_locations[i])

        total_mission_pts = len(mission_locations)
        data["total_mission_pts"] = len(_locations)
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        start_time = int(1537)
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft, ugv_stops_distance_dict)
        mid_pt_dist = ugv_distance_calc((5.35*5280, 5.95*5280), stp1_ugv_stop_ft, ugv_stops_distance_dict)
        ugv_travel_time = (dist // int(ugv_vel)) + 926
        velocity = int(ugv_vel)
        nw_stop_tw_1 = (mid_pt_dist // int(depot_to_first_stop_vel)) + start_time  # this tells the time at which UGV arrives NW/SE stop from depot
        mid_pt_to_nw_velocity = int(depot_to_first_stop_vel)
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        print(mid_pt_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), (5.35*5280, 5.95*5280), ugv_stops_distance_dict)
        return_dist_to_depotB = ugv_distance_calc((5.35*5280, 5.95*5280), (0.0*5280, 0.0*5280), ugv_stops_distance_dict)
        return_time_1 = se_stop_tw_2 + (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotB//mid_pt_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotB//mid_pt_to_nw_velocity) + start_time
        ugv_time_windows = ([(start_time, start_time+60)] +  # 0 (start)
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 1-6
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 7-12
                            [(0, veh_max_time)] * (n_depot_2 - 1) +  # 13-17
                            [(return_time_1, veh_max_time)] +  # 18
                            [(0, veh_max_time)] * total_mission_pts
                            )
        print(ugv_time_windows)

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity/3.281, mid_pt_to_nw_velocity/3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
        data["visits"] = [i for i in range(19, len(_locations))]
        print(data["stations"])
        print(data["visits"])
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, mid_pt_to_nw_velocity, velocity, pen, time_windows_dict
    elif strt_pt == 4:
        print("Start point is: {}".format(strt_pt))
        n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(0.36, 0.4), (0.71, 0.79), (1.07, 1.19), (1.43, 1.59), (1.78, 1.98), (2.14, 2.38),
                                         (2.5, 2.77), (2.86, 3.17), (3.21, 3.57), (3.57, 3.96), (3.93, 4.36), (4.28, 4.76), (4.64, 5.15), (5, 5.55),
                                         (5.35, 5.95)]
        depotA = [(5.35, 6.1), (5.35, 6.26), (5.34, 6.42),
                  (5.34, 6.58), (5.34, 6.74), (5.33, 6.9), (5.33, 7.05), (5.33, 7.21), (5.32, 7.37), (5.32, 7.53), (5.32, 7.69),
                  (5.31, 7.85), (5.31, 8.01), (5.31, 8.16), (5.3, 8.32), (5.3, 8.48)]
        depotA = [ele for ele in reversed(depotA)]

        for m in range(len(depotA)):
            ordered_mission_pts_stp1_stop.append(depotA)

        stp_1_list = [(0.36, 0.4), (0.71, 0.79), (1.07, 1.19), (1.43, 1.59), (1.78, 1.98), (2.14, 2.38),
                      (2.5, 2.77), (2.86, 3.17), (3.21, 3.57), (3.57, 3.96), (3.93, 4.36), (4.28, 4.76), (4.64, 5.15), (5, 5.55),
                      (5.35, 5.95)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(5.56, 5.89), (5.77, 5.83), (5.97, 5.77), (6.18, 5.71), (6.39, 5.65), (6.59, 5.59),
                                         (6.8, 5.53), (7.01, 5.47), (7.21, 5.41), (7.42, 5.35), (7.63, 5.3), (7.83, 5.24), (8.04, 5.18),
                                         (8.25, 5.12), (8.45, 5.06), (8.66, 5)]
        print(len(ordered_mission_pts_stp2_stop))

        ugv_stops_dict = scenario_2_data_processing_2stops.scenario_1_data_processing()

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
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(5.30, 8.48)] +
                      [(5.30, 8.48)] * n_depot_1 +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.0, 0.0)] * (n_depot_2)
                      # [(5.30, 8.48)]
                      )
        mission_locations = []
        # count = 0
        count_se = 0
        for i in range(len(stp_1_list)):
            mission_locations.append(stp_1_list[i])

        for i in range(len(ordered_mission_pts_stp2_stop)):
            count_se += 1
            if ordered_mission_pts_stp2_stop[i] == stp2_ugv_stop:
                for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
                    mission_locations.append(ordered_mission_pts_stp2_stop[j])
                break
            elif ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
                for j in range(count_se, len(ordered_mission_pts_stp2_stop)):
                    mission_locations.append(ordered_mission_pts_stp2_stop[j])
                break
            else:
                continue
        for i in range(len(mission_locations)):
            _locations.append(mission_locations[i])

        total_mission_pts = len(mission_locations)
        data["total_mission_pts"] = len(_locations)
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft, ugv_stops_distance_dict)
        depot_a_dist = ugv_distance_calc((5.30*5280, 8.48*5280), stp1_ugv_stop_ft, ugv_stops_distance_dict)
        ugv_travel_time = (dist // int(ugv_vel)) + 926
        velocity = int(ugv_vel)
        nw_stop_tw_1 = depot_a_dist // int(depot_to_first_stop_vel)  # this tells the time at which UGV arrives NW/SE stop from depot
        depot_a_to_nw_velocity = int(depot_to_first_stop_vel)
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        print(depot_a_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        return_dist_from_se = 0
        return_dist_to_depotA = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp2_ugv_stop_ft, ugv_stops_distance_dict)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp2_ugv_stop_ft, (5.30*5280, 8.48*5280), ugv_stops_distance_dict)
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, (5.30*5280, 8.48*5280), ugv_stops_distance_dict)
                break
            elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft, ugv_stops_distance_dict)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp2_ugv_stop_ft, (5.30*5280, 8.48*5280), ugv_stops_distance_dict)
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, (5.30*5280, 8.48*5280), ugv_stops_distance_dict)
                break
        return_time_1 = (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotA//depot_a_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
        ugv_time_windows = ([(0, 60)] +  # 0 (start)
                            [(0, veh_max_time)] * n_depot_1 +  # 1-6
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 7-12
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 13-18
                            [(0, veh_max_time)] * n_depot_2 +  # 19-24
                            [(0, veh_max_time)] * total_mission_pts
                            )
        print(ugv_time_windows)

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity/3.281, depot_a_to_nw_velocity/3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, depot_a_to_nw_velocity, velocity, pen, time_windows_dict
    else:
        print("Start point is: {}".format(strt_pt))
        print("Parameters are: {}, {}, {}".format(ugv_stop_loc, NW_stop, SE_stop))
        n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(5.35, 6.1), (5.35, 6.26), (5.34, 6.42),
                                         (5.34, 6.58), (5.34, 6.74), (5.33, 6.9), (5.33, 7.05), (5.33, 7.21), (5.32, 7.37), (5.32, 7.53), (5.32, 7.69),
                                         (5.31, 7.85), (5.31, 8.01), (5.31, 8.16), (5.3, 8.32), (5.3, 8.48)]
        ordered_mission_pts_stp1_stop = [ele for ele in reversed(ordered_mission_pts_stp1_stop)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(5.56, 5.89), (5.77, 5.83), (5.97, 5.77), (6.18, 5.71), (6.39, 5.65), (6.59, 5.59),
                                         (6.8, 5.53), (7.01, 5.47), (7.21, 5.41), (7.42, 5.35), (7.63, 5.3), (7.83, 5.24), (8.04, 5.18),
                                         (8.25, 5.12), (8.45, 5.06), (8.66, 5), (0.36, 0.4), (0.71, 0.79), (1.07, 1.19), (1.43, 1.59), (1.78, 1.98), (2.14, 2.38),
                                         (2.5, 2.77), (2.86, 3.17), (3.21, 3.57), (3.57, 3.96), (3.93, 4.36), (4.28, 4.76), (4.64, 5.15), (5, 5.55),
                                         (5.35, 5.95)]
        print(len(ordered_mission_pts_stp2_stop))
        stp_2_list = [(0.36, 0.4), (0.71, 0.79), (1.07, 1.19), (1.43, 1.59), (1.78, 1.98), (2.14, 2.38),
                      (2.5, 2.77), (2.86, 3.17), (3.21, 3.57), (3.57, 3.96), (3.93, 4.36), (4.28, 4.76), (4.64, 5.15), (5, 5.55),
                      (5.35, 5.95)]
        start_loc = (8.66*5280, 5*5280)
        ugv_stops_dict = scenario_2_data_processing_2stops.scenario_1_data_processing()

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
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(8.66, 5.0)] +
                      [(8.66, 5.0)] * n_depot_1 +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.0, 0.0)] * (n_depot_2)
                      # [(8.66, 5.0)]
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
            elif ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                for j in range(count-1):
                    mission_locations.append(ordered_mission_pts_stp1_stop[j])
                break
            else:
                continue

        for i in range(len(stp_2_list)):
            mission_locations.append(stp_2_list[i])

        for i in range(len(mission_locations)):
            _locations.append(mission_locations[i])

        total_mission_pts = len(mission_locations)
        data["total_mission_pts"] = len(_locations)
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft, ugv_stops_distance_dict)
        depot_c_dist = ugv_distance_calc((8.66*5280, 5*5280), stp1_ugv_stop_ft, ugv_stops_distance_dict)
        ugv_travel_time = (dist // int(ugv_vel)) + 926
        velocity = int(ugv_vel)
        nw_stop_tw_1 = depot_c_dist // int(depot_to_first_stop_vel)  # this tells the time at which UGV arrives NW/SE stop from depot
        depot_c_to_nw_velocity = int(depot_to_first_stop_vel)
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        print(depot_c_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        return_dist_from_se = 0
        return_dist_to_depotC = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft, ugv_stops_distance_dict)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, (8.66*5280, 5*5280), ugv_stops_distance_dict)
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp2_ugv_stop_ft, (8.66*5280, 5*5280), ugv_stops_distance_dict)
                break
            elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp2_ugv_stop_ft, ugv_stops_distance_dict)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, (8.66*5280, 5*5280), ugv_stops_distance_dict)
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp2_ugv_stop_ft, (8.66*5280, 5*5280), ugv_stops_distance_dict)
                break
        return_time_1 = (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotC//depot_c_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotC//depot_c_to_nw_velocity)
        ugv_time_windows = ([(0, 60)] +  # 0 (start)
                            [(0, veh_max_time)] * n_depot_1 +  # 1-6
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 7-12
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 13-18
                            [(0, veh_max_time)] * n_depot_2 +  # 19-24
                            [(0, veh_max_time)] * total_mission_pts
                            )
        print(ugv_time_windows)

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity/3.281, depot_c_to_nw_velocity/3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
                           [1] * total_mission_pts)  # 18-39
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
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, depot_c_to_nw_velocity, velocity, pen, time_windows_dict


def solution_route_update(data, ugv_stop_loc, strt_pt, ugv_stops_distance_dict, time_windows_dict, uav_time, rechrg_at_strt_pt):
    ordered_mission_pts_stp1_stop = [(5.35, 6.1), (5.35, 6.26), (5.34, 6.42),
                                     (5.34, 6.58), (5.34, 6.74), (5.33, 6.9), (5.33, 7.05), (5.33, 7.21), (5.32, 7.37), (5.32, 7.53), (5.32, 7.69),
                                     (5.31, 7.85), (5.31, 8.01), (5.31, 8.16), (5.3, 8.32), (5.3, 8.48)]
    ordered_mission_pts_stp1_stop = [ele for ele in reversed(ordered_mission_pts_stp1_stop)]

    ordered_mission_pts_stp2_stop = [(5.56, 5.89), (5.77, 5.83), (5.97, 5.77), (6.18, 5.71), (6.39, 5.65), (6.59, 5.59),
                                     (6.8, 5.53), (7.01, 5.47), (7.21, 5.41), (7.42, 5.35), (7.63, 5.3), (7.83, 5.24), (8.04, 5.18),
                                     (8.25, 5.12), (8.45, 5.06), (8.66, 5)]
    ugv_stops_dict = scenario_2_data_processing_2stops.scenario_1_data_processing()
    uav_addnl_time = 0
    ugv_addnl_time = 0
    current_uav_time = uav_time + rechrg_at_strt_pt
    ugv_wait_time = 0
    recharge_time = 900
    ugv_speed = 13
    if strt_pt == 4:
        if ugv_stops_dict[int(ugv_stop_loc)][1] in ordered_mission_pts_stp1_stop:
            dist = euclidean_distance((0*5280, 0*5280), (ugv_stops_dict[int(ugv_stop_loc)][1][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][1][1]*5280))
            uav_time_taken_from_depot_to_stop = dist // data["vehicle_speed"]
            current_uav_time += uav_time_taken_from_depot_to_stop
            if current_uav_time < time_windows_dict[13][0]:
                diff = (time_windows_dict[13][0] - current_uav_time)
                uav_time_taken_from_depot_to_stop += diff
            uav_time_to_rchg = uav_time_taken_from_depot_to_stop + recharge_time
            dist_to_dest = euclidean_distance((ugv_stops_dict[int(ugv_stop_loc)][1][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][1][1]*5280), (8.91*5280, 4.54*5280))
            uav_addnl_time = uav_time_to_rchg + dist_to_dest // data["vehicle_speed"]
            if current_uav_time < time_windows_dict[13][0]:
                ugv_addnl_time = recharge_time
            else:
                ugv_addnl_time = uav_addnl_time
        if ugv_stops_dict[int(ugv_stop_loc)][0] in ordered_mission_pts_stp1_stop:
            dist = euclidean_distance((0*5280, 0*5280), (ugv_stops_dict[int(ugv_stop_loc)][0][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][0][1]*5280))
            uav_time_taken_from_depot_to_stop = dist // data["vehicle_speed"]
            current_uav_time += uav_time_taken_from_depot_to_stop
            if current_uav_time < time_windows_dict[7][0]:
                diff = (time_windows_dict[7][0] - current_uav_time)
                uav_time_taken_from_depot_to_stop += diff
            elif current_uav_time > time_windows_dict[7][0]:
                dist_between_stops = ugv_distance_calc((ugv_stops_dict[int(ugv_stop_loc)][0][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][0][1]*5280), (ugv_stops_dict[int(ugv_stop_loc)][1][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][1][1]*5280), ugv_stops_distance_dict)
                ugv_wait_time = uav_time_taken_from_depot_to_stop - (time_windows_dict[13][1] + (dist_between_stops // ugv_speed))
            if current_uav_time > time_windows_dict[13][1]:
                ugv_wait_time = current_uav_time - (time_windows_dict[13][1])
            uav_time_to_rchg = uav_time_taken_from_depot_to_stop + recharge_time
            dist_to_dest = euclidean_distance((ugv_stops_dict[int(ugv_stop_loc)][0][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][0][1]*5280), (8.91*5280, 4.54*5280))
            uav_addnl_time = uav_time_to_rchg + dist_to_dest // data["vehicle_speed"]
            ugv_addnl_time = recharge_time
    if strt_pt == 5:
        if ugv_stops_dict[int(ugv_stop_loc)][1] in ordered_mission_pts_stp2_stop:
            dist = euclidean_distance((0*5280, 0*5280), (ugv_stops_dict[int(ugv_stop_loc)][1][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][1][1]*5280))
            uav_time_taken_from_depot_to_stop = dist // data["vehicle_speed"]
            current_uav_time += uav_time_taken_from_depot_to_stop
            if current_uav_time < time_windows_dict[13][0]:
                diff = (time_windows_dict[13][0] - current_uav_time)
                uav_time_taken_from_depot_to_stop += diff
            uav_time_to_rchg = uav_time_taken_from_depot_to_stop + recharge_time
            dist_to_dest = euclidean_distance((ugv_stops_dict[int(ugv_stop_loc)][1][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][1][1]*5280), (8.91*5280, 4.54*5280))
            uav_addnl_time = uav_time_to_rchg + dist_to_dest // data["vehicle_speed"]
            if current_uav_time < time_windows_dict[13][0]:
                ugv_addnl_time = recharge_time
            else:
                ugv_addnl_time = uav_addnl_time
        if ugv_stops_dict[int(ugv_stop_loc)][0] in ordered_mission_pts_stp2_stop:
            dist = euclidean_distance((0*5280, 0*5280), (ugv_stops_dict[int(ugv_stop_loc)][0][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][0][1]*5280))
            uav_time_taken_from_depot_to_stop = dist // data["vehicle_speed"]
            current_uav_time += uav_time_taken_from_depot_to_stop
            if current_uav_time < time_windows_dict[7][0]:
                diff = (time_windows_dict[7][0] - current_uav_time)
                uav_time_taken_from_depot_to_stop += diff
            elif current_uav_time > time_windows_dict[7][0]:
                dist_between_stops = ugv_distance_calc((ugv_stops_dict[int(ugv_stop_loc)][0][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][0][1]*5280), (ugv_stops_dict[int(ugv_stop_loc)][1][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][1][1]*5280), ugv_stops_distance_dict)
                ugv_wait_time = current_uav_time - (time_windows_dict[13][1] + (dist_between_stops // ugv_speed))
            if current_uav_time > time_windows_dict[13][1]:
                ugv_wait_time = current_uav_time - (time_windows_dict[13][1])
            uav_time_to_rchg = uav_time_taken_from_depot_to_stop + recharge_time
            dist_to_dest = euclidean_distance((ugv_stops_dict[int(ugv_stop_loc)][0][0]*5280, ugv_stops_dict[int(ugv_stop_loc)][0][1]*5280), (8.91*5280, 4.54*5280))
            uav_addnl_time = uav_time_to_rchg + dist_to_dest // data["vehicle_speed"]
            ugv_addnl_time = ugv_wait_time + recharge_time
    return uav_addnl_time, ugv_addnl_time


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


def euclidean_distance(position_1, position_2):  # This function calculates the distance UAV travels from one mission point to another
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])))


def print_solution(data, manager, routing, solution, ugv_stop_loc, strt_pt, ugv_stops_distance_dict, time_windows_dict):  # Prints the UAV optimization solution output on the console.
    print("Objective: {}".format(solution.ObjectiveValue()))
    total_distance = 0
    total_fuel = 0
    total_time = 0
    rechrg_at_strt_time = 0
    uav_additional_time = 0
    ugv_additional_time = 0
    distance_dimension = routing.GetDimensionOrDie("Distance")
    fuel_dimension = routing.GetDimensionOrDie("Fuel")
    time_dimension = routing.GetDimensionOrDie("Time")
    dropped_nodes = "Dropped nodes:"
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += " {}".format(manager.IndexToNode(node))
    print(dropped_nodes)
    dum_list = [(data["locations"][0][0], data["locations"][0][1])]
    dum_list2 = [(data["locations"][0][0], data["locations"][0][1])]
    coord_list = [(data["coordinates"][0][0], data["coordinates"][0][1])]
    time_elapsed = []
    fuel_remaining = []
    distance_travel = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        distance = 0
        while not routing.IsEnd(index):
            dist_var = distance_dimension.CumulVar(index)
            fuel_var = fuel_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            fuel_slack = fuel_dimension.SlackVar(index)
            plan_output += "{0} Fuel({1}) Time({2},{3}) Slack({4}) FuelSlack({5},{6}) Distance({7}) -> ".format(
                manager.IndexToNode(index),
                data["fuel_capacity"] - solution.Value(fuel_var),
                solution.Min(time_var), solution.Max(time_var),
                solution.Value(slack_var), solution.Min(fuel_slack), solution.Max(fuel_slack), solution.Value(dist_var))
            time_elapsed.append(round((solution.Min(time_var))/60))
            fuel_remaining.append(solution.Value(fuel_var))
            distance_travel.append(solution.Value(dist_var))
            previous_index = index
            if manager.IndexToNode(index) not in [1, 2, 3, 4, 5, 6, 19, 20, 21, 22, 23] and (solution.Value(routing.NextVar(index)) == routing.End(vehicle_id)):
                rechrg_at_strt_time = 900
            index = solution.Value(routing.NextVar(index))
            if vehicle_id == 0:
                print(data["total_mission_pts"])
                if index <= data["total_mission_pts"] - 2:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7 \
                            or index == 8 or index == 9 or index == 10 or index == 11 or index == 12 or index == 13 or index == 14 or index == 15 \
                            or index == 16 or index == 17 or index == 18 or index == 19 or index == 20 or index == 21 or index == 22 or index == 23:
                        dum_list.append(data["locations"][index])
                        coord_list.append(data["coordinates"][index])
                    else:
                        dum_list.append(data["locations"][index + 1])
                        coord_list.append(data["coordinates"][index + 1])
            else:
                if index <= data["total_mission_pts"] - 2:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7 \
                            or index == 8 or index == 9 or index == 10 or index == 11 or index == 12 or index == 13 or index == 14 or index == 15 \
                            or index == 16 or index == 17:
                        dum_list2.append(data["locations"][index])
                    else:
                        dum_list2.append(data["locations"][index + 1])
            # print(f"plop {routing.GetArcCostForVehicle(previous_index, index, vehicle_id)}\n")
            # print(f"plop {solution.Value(dist_var)}\n")
            # distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        dist_var = distance_dimension.CumulVar(index)
        fuel_var = fuel_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += "{0} Fuel({1}) Time({2},{3}) Distance({4})\n".format(
            manager.IndexToNode(index),
            data["fuel_capacity"] - solution.Value(fuel_var),
            solution.Min(time_var), solution.Max(time_var),
            solution.Value(dist_var))
        time_elapsed.append(round((solution.Min(time_var))/60))
        fuel_remaining.append(data["fuel_capacity"] - solution.Value(fuel_var))
        distance_travel.append(solution.Value(dist_var))
        uav_additional_time, ugv_additional_time = solution_route_update(data, ugv_stop_loc, strt_pt, ugv_stops_distance_dict, time_windows_dict, solution.Value(time_var), rechrg_at_strt_time)
        plan_output += "Distance of the route: {} ft\n".format(solution.Value(dist_var))
        plan_output += "Remaining Fuel of the route: {}\n".format(data["fuel_capacity"] - solution.Value(fuel_var))
        plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
        print(plan_output)
        total_distance += solution.Value(dist_var)
        total_fuel += data['fuel_capacity'] - solution.Value(fuel_var)
        total_time += solution.Value(time_var)
        total_time += uav_additional_time
    print('Total Distance of all routes: {} ft'.format(total_distance))
    print('Total Fuel remaining of all routes: {}'.format(total_fuel))
    print('Total Time of all routes: {} seconds'.format(total_time))
    dum_list.append((data["locations"][data["n_sum_rch_stops"]][0], data["locations"][data["n_sum_rch_stops"]][1]))
    dum_list2.append((data["locations"][data["n_sum_rch_stops"]][0], data["locations"][data["n_sum_rch_stops"]][1]))
    coord_list.append((data["coordinates"][data["n_sum_rch_stops"]][0], data["coordinates"][data["n_sum_rch_stops"]][1]))
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    for i in dum_list:
        x1.append(i[0])
        y1.append(i[1])
    for k in dum_list2:
        x2.append(k[0])
        y2.append(k[1])
    df2 = pd.DataFrame(coord_list, columns=['x', 'y'])
    df2.insert(0, 'time', time_elapsed)
    df2.insert(3, 'fuel remaining', fuel_remaining)
    # df2.to_excel('Scenario 1 GA dataset/plotting optimized parameter data_scn1_midptstrtpt.xlsx', index=False, engine='openpyxl')
    # if j == 7 and alpha == 1:
    #     df2 = pd.DataFrame(coord_list, columns=['x', 'y'])
    #     # df2.insert(0, 'time', time_elapsed)
    #     # df2.insert(3, 'fuel remaining', fuel_remaining)
    #     df2.to_excel('GA generation dataset/plotting optimized parameter data_'+str(alpha)+'_.xlsx', index=False, engine='openpyxl')
    # if j == 7 and alpha == 0.75:
    #     df2 = pd.DataFrame(coord_list, columns=['x', 'y'])
    #     # df2.insert(0, 'time', time_elapsed)
    #     # df2.insert(3, 'fuel remaining', fuel_remaining)
    #     df2.to_excel('GA generation dataset/plotting optimized parameter data_'+str(alpha)+'_.xlsx', index=False, engine='openpyxl')
    # if j == 7 and alpha == 0.5:
    #     df2 = pd.DataFrame(coord_list, columns=['x', 'y'])
    #     # df2.insert(0, 'time', time_elapsed)
    #     # df2.insert(3, 'fuel remaining', fuel_remaining)
    #     df2.to_excel('GA generation dataset/plotting optimized parameter data_'+str(alpha)+'_.xlsx', index=False, engine='openpyxl')
    # if j == 7 and alpha == 0:
    #     df2 = pd.DataFrame(coord_list, columns=['x', 'y'])
    #     # df2.insert(0, 'time', time_elapsed)
    #     # df2.insert(3, 'fuel remaining', fuel_remaining)
    #     df2.to_excel('GA generation dataset/plotting optimized parameter data_'+str(alpha)+'_.xlsx', index=False, engine='openpyxl')
    return solution.ObjectiveValue(), total_distance, total_time, distance_travel, uav_additional_time, ugv_additional_time


def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    route = {}
    time_dimension = routing.GetDimensionOrDie("Time")
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route[manager.IndexToNode(index)] = 0
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            route[manager.IndexToNode(index)] = solution.Min(time_var)
        routes.append(route)
    return route, routes


def main(ugv_stop_loc, NW_stop, SE_stop, strt_pt, ugv_stops_distance_dict):  # This is where optimization of UAV routes happen.
    """Entry point of the program."""
    # Instantiate the data problem.
    # [START data]
    print("----------------------------------------------------------------------")
    print("***************UAV optimization via feedback solution*****************")
    print("----------------------------------------------------------------------")
    data, veh_max_time, depotb_vel, ugv_vel, pen, tw_dict = create_data_model(ugv_stop_loc, NW_stop, SE_stop, strt_pt, ugv_stops_distance_dict)
    # [END data]

    # Create the routing index manager.
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(
        len(data['time_windows']),
        data['num_vehicles'],
        data['starts'],
        data['ends'])
    # [END index_manager]
    starts = data["starts"]
    ends = data["ends"]

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)
    # [END routing_model]

    # Distance
    stations = data["stations"]
    visits = data["visits"]
    # period_2_nodes = [i for i in range(58, 79)]  # run the program now!
    #[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # depot + refill stations

    solution_graph = list()
    start_time = time()

    def solution_callback():
        t = time() - start_time
        solution_graph.append((t, routing.CostVar().Max()))
    routing.AddAtSolutionCallback(solution_callback)
    # print(solution_graph)
    # figure2 = plt.figure()
    # ax2 = figure2.add_subplot(111)
    # for i in range(len(solution_graph)):
    #     ax2.plot(solution_graph[i][0], solution_graph[i][1])

    # routing.AddConstantDimension(1, manager.GetNumberOfNodes(), True,
    #                              "Counter")
    # counter_dimension = routing.GetDimensionOrDie("Counter")
    # nb_visit = 62 // manager.GetNumberOfVehicles()
    # print(f'visit_mean: {nb_visit}')

    # for vehicle_id in range(data["num_vehicles"]):
    #     index = routing.End(vehicle_id)
    #     counter_dimension.SetCumulVarSoftLowerBound(index, nb_visit, 100)
    #     counter_dimension.SetCumulVarSoftUpperBound(index, nb_visit + 1, 100)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # if from_node in stations and to_node in stations:
        #     return data["fuel_capacity"]*5
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    #
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        300000000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)

    # Create and register a time callback.
    # [START time_callback]
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # value = 0
        if from_node in stations and to_node in stations:
            return data["fuel_capacity"] * 5
        # elif from_node in stations and to_node in ends:
        #     value = data["fuel_capacity"] * 10
        else:
            return int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"])
        #print(f'DEBUG: time({from_node},{to_node}): {value}')

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)
    # [END time_callback]

    routing.AddDimension(
        time_callback_index,
        data["horizon"],  # max slack/wait time
        data["horizon"],  # vehicle max time
        False,  # don't force cumul to zero
        'Time')

    time_dimension = routing.GetDimensionOrDie('Time')
    # time_dimension.SetGlobalSpanCostCoefficient(100)
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx in data["starts"] or location_idx in data["ends"]:
            continue
        index = manager.NodeToIndex(location_idx)
        routing.AddToAssignment(time_dimension.SlackVar(index))
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    # and "copy" the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0],
                                                data["time_windows"][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # depot_ends = data['ends']
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.End(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][data["n_sum_rch_stops"]][0],
                                                data["time_windows"][data["n_sum_rch_stops"]][1])

    for i in range(manager.GetNumberOfVehicles()):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    def interval_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return math.ceil(((((data["distance_matrix"][from_node][to_node] / (data["vehicle_speed"]*60))) ** 2)/900))

    interval_callback_index = routing.RegisterTransitCallback(interval_callback)
    # routing.SetArcCostEvaluatorOfAllVehicles(interval_callback_index)

    # Fuel constraints

    def fuel_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        value = 0
        if from_node in stations and from_node not in starts:
            value = -data['fuel_capacity'] - int(198 * ((data["distance_matrix"][from_node][to_node]) / data["vehicle_speed"]))
        value += int(198 * ((data["distance_matrix"][from_node][to_node]) / data["vehicle_speed"]))
        # print(f'DEBUG: fuel({from_node},{to_node}): {value}')
        return value

    fuel_callback_index = routing.RegisterTransitCallback(fuel_callback)
    # routing.SetArcCostEvaluatorOfAllVehicles(interval_callback_index)
    routing.AddDimension(
        fuel_callback_index,
        data["fuel_capacity"],
        data["fuel_capacity"],
        False,
        'Fuel')

    fuel_dimension = routing.GetDimensionOrDie('Fuel')
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        fuel_dimension.SlackVar(routing.Start(vehicle_id)).SetValue(0)
        routing.AddToAssignment(fuel_dimension.SlackVar(routing.Start(vehicle_id)))
        for node in range(len(data["distance_matrix"])):
            if node in starts or node in ends:
                continue
            if node > data["n_sum_rch_stops"]:
                index = manager.NodeToIndex(node)
                fuel_dimension.SlackVar(index).SetValue(0)
                # routing.AddVariableMinimizedByFinalizer(fuel_dimension.CumulVar(node))
            # else:  # station node
            # pass
            index = manager.NodeToIndex(node)
            routing.AddToAssignment(fuel_dimension.SlackVar(index))
            # fuel_dimension.CumulVar(index).SetValue(0)

    # def augmented_objective_function(from_index, to_index):
    #     energy_cost = distance_callback(from_index, to_index)
    #     weighted_time_cost = time_callback(from_index, to_index)
    #     return (alpha * weighted_time_cost) + ((1 - alpha) * energy_cost)
    # #
    # augmented_fn_callback_index = routing.RegisterTransitCallback(augmented_objective_function)
    # routing.SetArcCostEvaluatorOfAllVehicles(augmented_fn_callback_index)

    for station_node in stations:
        for visit_node in visits:
            if station_node in starts or station_node in ends:
                continue
            if station_node in stations and visit_node in visits:
                energy = int(198 * ((data["distance_matrix"][station_node][visit_node]) / data["vehicle_speed"]))
                station_index = manager.NodeToIndex(station_node)
                visit_index = manager.NodeToIndex(visit_node)
                cond = routing.NextVar(station_index) == visit_index
                routing.solver().Add(cond * fuel_dimension.CumulVar(visit_index) == cond * energy)

    for vehicle_id in range(data["num_vehicles"]):
        routing.solver().Add(time_dimension.SlackVar(routing.Start(vehicle_id)) <= 60)
        for from_node in range(len(data["distance_matrix"])):
            for to_node in range(len(data["distance_matrix"])):
                if from_node == 0 or from_node == data["n_sum_rch_stops"]:
                    continue
                if from_node < data["n_sum_rch_stops"] and to_node > data["n_sum_rch_stops"]:
                    from_index = manager.NodeToIndex(from_node)
                    to_index = manager.NodeToIndex(to_node)
                    tmp = (((fuel_dimension.CumulVar(from_index))) <= 17300)
                    routing.solver().Add(time_dimension.SlackVar(from_index) == (tmp * 60) + ((1 - tmp) * ((fuel_dimension.CumulVar(from_index)) // 311)))
                if from_node and to_node > data["n_sum_rch_stops"]:
                    to_index = manager.NodeToIndex(to_node)
                    routing.solver().Add(time_dimension.SlackVar(to_index) <= 120)

    visit_penalty = 1_000_000
    station_penalty = 0
    for node in range(len(data["distance_matrix"])):
        if node in starts or node in ends:
            continue
        index = manager.NodeToIndex(node)
        if node in stations:
            routing.AddDisjunction([index], station_penalty)
        else:  # regular location
            routing.AddDisjunction([index], visit_penalty)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # search_parameters.log_search = True
    search_parameters.time_limit.FromSeconds(15)

    # Solve the problem.
    # [START solve]
    solution = routing.SolveWithParameters(search_parameters)
    # routing.AddAtSolutionCallback(interval_callback)
    # [END solve]

    # Print solution on console.
    # [START print_solution]
    # ugv_tw_array = np.array(data["time_windows"])
    if solution:
        obj_val, total_dist, total_time, disttravel_track, uav_addn_time, ugv_addn_time  = print_solution(data, manager, routing, solution, ugv_stop_loc, strt_pt, ugv_stops_distance_dict, tw_dict)
        route_dict, route = get_routes(solution, routing, manager)
        veh_max_time += ugv_addn_time
        if obj_val < 1_000_000:
            print(f'Objective value is: {veh_max_time - total_time}')
            return veh_max_time - total_time, veh_max_time, total_dist, total_time, depotb_vel, ugv_vel, pen, route_dict, route, tw_dict, disttravel_track
        else:
            return obj_val, veh_max_time, total_dist, total_time, depotb_vel, ugv_vel, pen, route_dict, route, tw_dict, disttravel_track
    else:
        print("*************No solution found************")
        return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, [0, 0]
    # [END print_solution]
    # print("Solver status:", routing.status())
    # print("The UAV optimization for current parameter setting is: {}".format(objective_value))
    # print("****************************************************\n")

# (74, 14, 14, 10, 4, 5) - our optimal solution for scenario 1


def ugv_distance_calc_2():
    ugv_stops_distance_dict = {}
    df = pd.read_csv('Scenario 2 data points.csv')
    df_list = df.values.tolist()
    for i in range(len(df_list)):
        df_list[i][0] = round(df_list[i][0], 2)
        df_list[i][1] = round(df_list[i][1], 2)
        df_list[i] = tuple(df_list[i])
    combo = combinations(df_list, 2)
    combo = list(combo)
    for i in range(len(combo)):
        combo[i] = list(combo[i])
    rev_ugv_stops_lst = copy.deepcopy(combo)
    # for r in range(len(ugv_Stops)):
    #     rev_ugv_stops_lst.append(ugv_Stops[r])

    for i in range(len(rev_ugv_stops_lst)):
        rev_ugv_stops_lst[i].reverse()

    for l in range(len(rev_ugv_stops_lst)):
        combo.append(rev_ugv_stops_lst[l])
    # print(combo)
    for j in range(len(combo)):
        combo[j] = tuple(combo[j])
        ugv_stops_distance_dict[combo[j]] = scenario_2_data_processing_2stops.ugv_distance(combo[j][0], combo[j][1])
    return ugv_stops_distance_dict


if __name__ == '__main__':
    ugv_stops_dict = ugv_distance_calc_2()
    graph = main(6, 5, 5, 5, ugv_stops_dict)
