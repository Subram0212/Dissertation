"""UGV travels at a speed of 4.3-4.5 m/s. UAV travels at a speed of 10 m/s.
Functions applied in this inner loop UAV optimization code are described as follows:

create_data_model: create a dictionary consisting of UAV and UGV parameters required for optimization.
ugv_distance_calc: This function returns the distance needed to be traveled by UGV between any two locations.
euclidean_distance: This function calculates the distance needed to be traveled by UAV between two mission points.
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


def create_data_model(ugv_stop_loc, NW_stop, SE_stop, strt_pt):
    n_depot_1 = 6
    n_stop_1 = 6
    n_stop_2 = 6
    n_depot_2 = 6
    data = {}
    fuel_capacity = 287700
    if strt_pt == 2:
        n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(4.29, 11.21), (4.01, 10.83), (3.74, 10.45),
                                         (3.54, 10.04), (3.42, 9.61), (3.29, 9.23), (3.01, 8.87), (2.61, 8.34), (2.43, 7.91), (2.77, 7.57), (3.1, 7.24),
                                         (3.43, 6.9), (3.8, 6.71), (4.11, 6.59), (4.55, 6.44), (4.66, 6.4), (4.98, 6.28), (5.37, 6.07),
                                         (5.77, 5.87)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(6.09, 5.64), (6.22, 5.25), (6.43, 4.86), (6.77, 4.53), (7.11, 4.19), (7.45, 3.86),
                                         (7.62, 3.69), (7.7, 3.61), (8.04, 3.28), (8.39, 2.98), (8.75, 2.73), (9.13, 2.5), (9.53, 2.29),
                                         (9.93, 2.09), (10.33, 1.88), (10.75, 1.79), (10.58, 1.58), (10.28, 1.28), (10.44, 0.96),
                                         (10.54, 0.90)]
        print(len(ordered_mission_pts_stp2_stop))

        ugv_stops_dict = {}
        ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)],
                     [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)],
                     [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)],
                     [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)], [(3.42, 9.61), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)],
                     [(3.54, 10.04), (10.54, 0.9)], [(3.74, 10.45), (10.54, 0.9)]]

        # ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)], [(3.1, 7.24), (10.44, 0.96)], [(3.1, 7.24), (10.54, 0.9)],
        #              [(2.77, 7.57), (7.7, 3.61)], [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.77, 7.57), (10.44, 0.96)], [(2.77, 7.57), (10.54, 0.9)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.43, 7.91), (10.44, 0.96)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (7.45, 3.86)],
        #              [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (8.39, 2.98)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(2.61, 8.34), (10.44, 0.96)], [(2.61, 8.34), (10.54, 0.9)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)], [(3.01, 8.87), (10.44, 0.96)], [(3.01, 8.87), (10.54, 0.9)],
        #              [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.29, 9.23), (9.53, 2.29)], [(3.29, 9.23), (10.44, 0.96)], [(3.29, 9.23), (10.54, 0.9)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.42, 9.61), (10.44, 0.96)], [(3.42, 9.61), (10.54, 0.9)],
        #              [(3.74, 10.45), (10.44, 0.96)], [(3.74, 10.45), (10.54, 0.9)], [(3.54, 10.04), (10.44, 0.96)], [(3.54, 10.04), (10.54, 0.9)]]

        # rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
        #
        # for i in range(len(rev_ugv_stops_lst)):
        #     rev_ugv_stops_lst[i].reverse()
        #
        # for l in range(len(rev_ugv_stops_lst)):
        #     ugv_Stops.append(rev_ugv_stops_lst[l])

        for i in range(len(ugv_Stops)):
            ugv_stops_dict[i+2] = ugv_Stops[i]

        stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
        stp1_ugv_stop_list_copy = []
        for i in range(len(stp1_ugv_stop_list)):
            stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
        stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
        stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
        stp2_ugv_stop_list_copy = []
        for i in range(len(stp2_ugv_stop_list)):
            stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
        stop1 = False
        stop2 = False
        if ugv_stops_dict[int(ugv_stop_loc)][1] == (10.54, 0.9):
            stop2 = True
        elif ugv_stops_dict[int(ugv_stop_loc)][0] == (10.54, 0.9):
            stop1 = True
        stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(0.6, 8.07)] +
                      [(0.6, 8.07)] * n_depot_1 +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.6, 8.07)] * n_depot_2
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
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        print(NW_stop, SE_stop)
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft)
        depot_b_dist = ugv_distance_calc((0.6*5280, 8.07*5280), stp1_ugv_stop_ft)
        ugv_travel_time = (dist // 13) + 926
        velocity = 13
        nw_stop_tw_1 = depot_b_dist // 13  # this tells the time at which UGV arrives NW/SE stop from depot
        depot_b_to_nw_velocity = 13
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        print(depot_b_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), (3.79*5280, 6.71*5280))
        return_dist_to_depotB = ugv_distance_calc((3.79*5280, 6.71*5280), (0.6*5280, 8.07*5280))
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
        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity//3.281, depot_b_to_nw_velocity//3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
        data["visits"] = [i for i in range(n_sum_rch_stops+1, len(_locations))]
        print(data["stations"])
        print(data["visits"])
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, depot_b_to_nw_velocity, velocity, pen, time_windows_dict, stop1, stop2
    elif strt_pt == 3:
        n_sum_rch_stops = n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(4.29, 11.21), (4.01, 10.83), (3.74, 10.45),
                                         (3.54, 10.04), (3.42, 9.61), (3.29, 9.23), (3.01, 8.87), (2.61, 8.34), (2.43, 7.91), (3.1, 7.24), (2.77, 7.57),
                                         (3.43, 6.9), (3.8, 6.71), (4.11, 6.59), (4.55, 6.44), (4.66, 6.4), (4.98, 6.28), (5.37, 6.07),
                                         (5.77, 5.87)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(6.09, 5.64), (6.22, 5.25), (6.43, 4.86), (6.77, 4.53), (7.11, 4.19), (7.45, 3.86),
                                         (7.62, 3.69), (7.7, 3.61), (8.04, 3.28), (8.39, 2.98), (8.75, 2.73), (9.13, 2.5), (9.53, 2.29),
                                         (9.93, 2.09), (10.33, 1.88), (10.75, 1.79), (10.58, 1.58), (10.28, 1.28), (10.44, 0.96),
                                         (10.54, 0.90)]
        print(len(ordered_mission_pts_stp2_stop))

        ugv_stops_dict = {}
        ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)],
                     [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)],
                     [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)],
                     [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (10.54, 0.9)],
                     [(3.74, 10.45), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.54, 10.04), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)]]

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

        stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
        stp1_ugv_stop_list_copy = []
        for i in range(len(stp1_ugv_stop_list)):
            stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
        stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
        stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
        stp2_ugv_stop_list_copy = []
        for i in range(len(stp2_ugv_stop_list)):
            stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
        stop1 = False
        stop2 = False
        stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(3.8, 6.71)] +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.6, 8.07)] * (n_depot_2 - 1) +
                      [(3.8, 6.71)]
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
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        start_time = int(round(random.uniform(1537, 4615), 0))
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft)
        mid_pt_dist = ugv_distance_calc((3.79*5280, 6.71*5280), stp1_ugv_stop_ft)
        ugv_travel_time = (dist // 13) + 926
        velocity = 13
        nw_stop_tw_1 = (mid_pt_dist // 13) + start_time  # this tells the time at which UGV arrives NW stop
        mid_pt_to_nw_velocity = 13
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        # nw_stop_return_tw_1 = se_stop_tw_2 + ugv_travel_time
        # nw_stop_return_tw_2 = se_stop_tw_2 + ugv_travel_time + 60
        print(mid_pt_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), (3.79*5280, 6.71*5280))
        return_dist_to_depotB = ugv_distance_calc((3.79*5280, 6.71*5280), (0.6*5280, 8.07*5280))
        return_time_1 = (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotB//mid_pt_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotB//mid_pt_to_nw_velocity) + start_time
        ugv_time_windows = ([(start_time, start_time+60)] +  # 0 (start)
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 3-5
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 6-12  # 13-14
                            [(0, veh_max_time)] * (n_depot_2 - 1) +  # 15-17
                            [(return_time_1, veh_max_time)] +
                            [(0, veh_max_time)] * total_mission_pts  # 18-39
                            )
        # first_nw_stop_duration = nw_stop_tw_2 - nw_stop_tw_1
        # se_stop_duration = se_stop_tw_2 - se_stop_tw_1
        # second_nw_stop_duration = nw_stop_return_tw_2 - nw_stop_return_tw_1
        # dur_list = [nw_ugv_stop, se_ugv_stop, nw_stop_tw_1//60, ugv_travel_time//60, first_nw_stop_duration//60, se_stop_duration//60, veh_max_time//60]
        # duration_dict[m+1] = dur_list

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity//3.281, mid_pt_to_nw_velocity//3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
                           [0] * (n_sum_rch_stops - 1) +  # 1-16 (stations)
                           [0] +  # 17 (end)
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
        data["visits"] = [i for i in range(n_sum_rch_stops+1, len(_locations))]
        print(data["stations"])
        print(data["visits"])
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, mid_pt_to_nw_velocity, velocity, pen, time_windows_dict, stop1, stop2
    elif strt_pt == 4:
        n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(0.63, 7.97), (0.75, 7.58), (1.2, 6.82), (1.54, 6.48), (2.44, 6.46), (3.37, 6.33), (3.74, 10.45),
                                         (3.54, 10.04), (3.42, 9.61), (3.29, 9.23), (3.01, 8.87), (2.61, 8.34), (2.43, 7.91), (3.1, 7.24), (2.77, 7.57),
                                         (3.43, 6.9), (3.8, 6.71), (4.11, 6.59), (4.55, 6.44), (4.66, 6.4), (4.98, 6.28), (5.37, 6.07),
                                         (5.77, 5.87)]
        print(len(ordered_mission_pts_stp1_stop))
        stp_1_list = [(0.63, 7.97), (0.75, 7.58), (1.2, 6.82), (1.54, 6.48), (2.44, 6.46), (3.37, 6.33)]

        ordered_mission_pts_stp2_stop = [(6.09, 5.64), (6.22, 5.25), (6.43, 4.86), (6.77, 4.53), (7.11, 4.19), (7.45, 3.86),
                                         (7.62, 3.69), (7.7, 3.61), (8.04, 3.28), (8.39, 2.98), (8.75, 2.73), (9.13, 2.5), (9.53, 2.29),
                                         (9.93, 2.09), (10.33, 1.88), (10.75, 1.79), (10.58, 1.58), (10.28, 1.28), (10.44, 0.96),
                                         (10.54, 0.90)]
        print(len(ordered_mission_pts_stp2_stop))

        ugv_stops_dict = {}
        ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)],
                     [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)],
                     [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)],
                     [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (10.54, 0.9)],
                     [(3.74, 10.45), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.54, 10.04), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)]]

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

        stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
        stp1_ugv_stop_list_copy = []
        for i in range(len(stp1_ugv_stop_list)):
            stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
        stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
        stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
        stp2_ugv_stop_list_copy = []
        for i in range(len(stp2_ugv_stop_list)):
            stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
        stop1 = False
        stop2 = False
        stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(4.29, 11.21)] +
                      [(4.29, 11.21)] * n_depot_1 +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.6, 8.07)] * (n_depot_2 - 1) +
                      [(4.29, 11.21)]
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
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft)
        depot_a_dist = ugv_distance_calc((4.29*5280, 11.21*5280), stp1_ugv_stop_ft)
        ugv_travel_time = (dist // 13) + 926
        velocity = 13
        nw_stop_tw_1 = depot_a_dist // 13  # this tells the time at which UGV arrives NW stop
        depot_a_to_nw_velocity = 13
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        # nw_stop_return_tw_1 = se_stop_tw_2 + ugv_travel_time
        # nw_stop_return_tw_2 = se_stop_tw_2 + ugv_travel_time + 60
        print(depot_a_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        # return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        return_dist_from_se = 0
        return_dist_to_depotA = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp2_ugv_stop_ft)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp2_ugv_stop_ft, (4.29*5280, 11.21*5280))
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, (4.29*5280, 11.21*5280))
                break
            elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp2_ugv_stop_ft, (4.29*5280, 11.21*5280))
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, (4.29*5280, 11.21*5280))
                break
        # for i in range(len(ordered_mission_pts_stp2_stop)):
        #     if ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
        #         return_dist_from_se_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        #         return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp2_ugv_stop_ft)
        #         if ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
        #             return_dist_to_depotA = ugv_distance_calc(stp1_ugv_stop_ft, (4.29*5280, 11.21*5280))
        #         elif ordered_mission_pts_stp2_stop[i] == stp2_ugv_stop:
        #             return_dist_to_depotA = ugv_distance_calc(stp2_ugv_stop_ft, (4.29*5280, 11.21*5280))
        #         break
        return_time_1 = (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotA//depot_a_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + return_time_2
        ugv_time_windows = ([(0, 60)] +  # 0 (start)
                            [(0, veh_max_time)] * n_depot_1 +  # 1-2
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 3-5
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 6-12  # 13-14
                            [(0, veh_max_time)] * n_depot_2 +  # 15-17
                            [(0, veh_max_time)] * total_mission_pts  # 18-39
                            )
        # first_nw_stop_duration = nw_stop_tw_2 - nw_stop_tw_1
        # se_stop_duration = se_stop_tw_2 - se_stop_tw_1
        # second_nw_stop_duration = nw_stop_return_tw_2 - nw_stop_return_tw_1
        # dur_list = [nw_ugv_stop, se_ugv_stop, nw_stop_tw_1//60, ugv_travel_time//60, first_nw_stop_duration//60, se_stop_duration//60, veh_max_time//60]
        # duration_dict[m+1] = dur_list

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity//3.281, depot_a_to_nw_velocity//3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
                           [0] * (n_sum_rch_stops - 1) +  # 1-16 (stations)
                           [0] +  # 17 (end)
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
        data["visits"] = [i for i in range(n_sum_rch_stops+1, len(_locations))]
        print(data["stations"])
        print(data["visits"])
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, depot_a_to_nw_velocity, velocity, pen, time_windows_dict, stop1, stop2
    else:
        n_sum_rch_stops = n_depot_1 + n_stop_1 + n_stop_2 + n_depot_2
        ordered_mission_pts_stp1_stop = [(4.29, 11.21), (4.01, 10.83), (3.74, 10.45),
                                         (3.54, 10.04), (3.42, 9.61), (3.29, 9.23), (3.01, 8.87), (2.61, 8.34), (2.43, 7.91), (3.1, 7.24), (2.77, 7.57),
                                         (3.43, 6.9), (3.8, 6.71), (4.11, 6.59), (4.55, 6.44), (4.66, 6.4), (4.98, 6.28), (5.37, 6.07),
                                         (5.77, 5.87)]
        print(len(ordered_mission_pts_stp1_stop))

        ordered_mission_pts_stp2_stop = [(6.09, 5.64), (6.22, 5.25), (6.43, 4.86), (6.77, 4.53), (7.11, 4.19), (7.45, 3.86),
                                         (7.62, 3.69), (7.7, 3.61), (8.04, 3.28), (8.39, 2.98), (8.75, 2.73), (9.13, 2.5), (9.53, 2.29),
                                         (0.63, 7.97), (0.75, 7.58), (1.2, 6.82), (1.54, 6.48), (2.44, 6.46), (3.37, 6.33)]
        print(len(ordered_mission_pts_stp2_stop))
        stp_2_list = [(0.63, 7.97), (0.75, 7.58), (1.2, 6.82), (1.54, 6.48), (2.44, 6.46), (3.37, 6.33)]

        ugv_stops_dict = {}
        ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)],
                     [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)],
                     [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)],
                     [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (10.54, 0.9)],
                     [(3.74, 10.45), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.54, 10.04), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)]]

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

        stp1_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][0]]
        stp1_ugv_stop_list_copy = []
        for i in range(len(stp1_ugv_stop_list)):
            stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
        stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
        stp2_ugv_stop_list = [ugv_stops_dict[int(ugv_stop_loc)][1]]
        stp2_ugv_stop_list_copy = []
        for i in range(len(stp2_ugv_stop_list)):
            stp2_ugv_stop_list_copy.append(stp2_ugv_stop_list[i])
        stop1 = False
        stop2 = False
        stp2_ugv_stop = stp2_ugv_stop_list_copy.pop()
        stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
        stp2_ugv_stp_ft_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
        stp2_ugv_stop_ft = tuple(stp2_ugv_stp_ft_list)
        print(stp1_ugv_stop, stp2_ugv_stop)

        _locations = ([(10.54, 0.9)] +
                      [(10.54, 0.9)] * n_depot_1 +
                      stp1_ugv_stop_list * n_stop_1 +
                      stp2_ugv_stop_list * n_stop_2 +
                      [(0.6, 8.07)] * (n_depot_2 - 1) +
                      [(10.54, 0.9)]
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
        data["coordinates"] = _locations
        data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
        data["num_locations"] = len(data["locations"])
        nw_wait_time = int(NW_stop) * 60
        se_wait_time = int(SE_stop) * 60
        dist = ugv_distance_calc(stp1_ugv_stop_ft, stp2_ugv_stop_ft)
        depot_c_dist = ugv_distance_calc((10.54*5280, 0.9*5280), stp1_ugv_stop_ft)
        ugv_travel_time = (dist // 13) + 926
        velocity = 13
        nw_stop_tw_1 = depot_c_dist // 13  # this tells the time at which UGV arrives NW stop
        depot_c_to_nw_velocity = 13
        nw_stop_tw_2 = nw_stop_tw_1 + nw_wait_time
        se_stop_tw_1 = nw_stop_tw_2 + ugv_travel_time
        se_stop_tw_2 = nw_stop_tw_2 + ugv_travel_time + se_wait_time
        # nw_stop_return_tw_1 = se_stop_tw_2 + ugv_travel_time
        # nw_stop_return_tw_2 = se_stop_tw_2 + ugv_travel_time + 60
        print(depot_c_to_nw_velocity, velocity)
        print("The travel time for UGV from depot B to NW UGV stop is: {}".format(nw_stop_tw_1))
        print("The UGV travel time between two UGV stops is: {}".format(ugv_travel_time))
        # return_dist_from_se_list = 0
        return_dist_from_se = 0
        return_dist_to_depotC = 0
        for i in range(len(ordered_mission_pts_stp1_stop)):
            if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp1_ugv_stop_ft)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, (10.54*5280, 0.9*5280))
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp2_ugv_stop_ft, (10.54*5280, 0.9*5280))
                break
            elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                return_dist_from_se_list = [stp2_ugv_stop[i]*5280 for i in range(len(stp2_ugv_stop))]
                return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp2_ugv_stop_ft)
                if ordered_mission_pts_stp1_stop[i] == stp2_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, (10.54*5280, 0.9*5280))
                elif ordered_mission_pts_stp1_stop[i] == stp1_ugv_stop:
                    return_dist_to_depotC = ugv_distance_calc(stp2_ugv_stop_ft, (10.54*5280, 0.9*5280))
                break
        # for i in range(len(ordered_mission_pts_stp2_stop)):
        #     if ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop or ordered_mission_pts_stp2_stop[i] == stp2_ugv_stop:
        #         return_dist_from_se_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
        #         return_dist_from_se = ugv_distance_calc(tuple(return_dist_from_se_list), stp2_ugv_stop_ft)
        #         if ordered_mission_pts_stp2_stop[i] == stp1_ugv_stop:
        #             return_dist_to_depotC = ugv_distance_calc(stp1_ugv_stop_ft, (10.54*5280, 0.9*5280))
        #         elif ordered_mission_pts_stp2_stop[i] == stp2_ugv_stop:
        #             return_dist_to_depotC = ugv_distance_calc(stp2_ugv_stop_ft, (10.54*5280, 0.9*5280))
        #         break
        return_time_1 = (return_dist_from_se//velocity)
        return_time_2 = (return_dist_to_depotC//depot_c_to_nw_velocity)
        veh_max_time = se_stop_tw_2 + (return_dist_from_se//velocity) + (return_dist_to_depotC//depot_c_to_nw_velocity)
        ugv_time_windows = ([(0, 60)] +  # 0 (start)
                            [(0, veh_max_time)] * n_depot_1 +  # 1-2
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 3-5
                            [(se_stop_tw_1, se_stop_tw_2)] * n_stop_2 +  # 6-12  # 13-14
                            [(0, veh_max_time)] * n_depot_2 +  # 15-17
                            [(0, veh_max_time)] * total_mission_pts  # 18-39
                            )
        # first_nw_stop_duration = nw_stop_tw_2 - nw_stop_tw_1
        # se_stop_duration = se_stop_tw_2 - se_stop_tw_1
        # second_nw_stop_duration = nw_stop_return_tw_2 - nw_stop_return_tw_1
        # dur_list = [nw_ugv_stop, se_ugv_stop, nw_stop_tw_1//60, ugv_travel_time//60, first_nw_stop_duration//60, se_stop_duration//60, veh_max_time//60]
        # duration_dict[m+1] = dur_list

        total_ugv_power = ugv_power_consumption.ugv_power(nw_wait_time, se_wait_time, velocity//3.281, depot_c_to_nw_velocity//3.281, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2)
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
                           [0] * (n_sum_rch_stops - 1) +  # 1-16 (stations)
                           [0] +  # 17 (end)
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
        data["visits"] = [i for i in range(n_sum_rch_stops+1, len(_locations))]
        print(data["stations"])
        print(data["visits"])
        time_windows_dict = {}
        for i, time_ in enumerate(data["time_windows"]):
            time_windows_dict[i] = time_
        return data, veh_max_time, depot_c_to_nw_velocity, velocity, pen, time_windows_dict, stop1, stop2


def ugv_distance_calc(stop_1, stop_2):  # distance will be in ft. Calculation of UGV travel distance between two mission points.
    if stop_1 == stop_2:
        dist = 0
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.1*5280, 7.24*5280):
        dist = 5.17*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (2.77*5280, 7.57*5280):
        dist = 5.64*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (2.43*5280, 7.91*5280):
        dist = 6.12*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (2.61*5280, 8.34*5280):
        dist = 6.58*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.01*5280, 8.87*5280):
        dist = 7.25*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.29*5280, 9.23*5280):
        dist = 7.70*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.42*5280, 9.61*5280):
        dist = 8.10*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (7.45*5280, 3.86*5280):
        dist = 9.12*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (7.7*5280, 3.61*5280):
        dist = 9.48*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (8.04*5280, 3.28*5280):
        dist = 9.95*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (8.39*5280, 2.98*5280):
        dist = 10.41*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (8.75*5280, 2.73*5280):
        dist = 10.85*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (9.13*5280, 2.5*5280):
        dist = 11.29*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (9.53*5280, 2.29*5280):
        dist = 11.74*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (10.54*5280, 0.9*5280):
        dist = 14.22*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (3.1*5280, 7.24*5280):
        dist = 0.88*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (2.77*5280, 7.57*5280):
        dist = 1.35*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (2.43*5280, 7.91*5280):
        dist = 1.83*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (2.61*5280, 8.34*5280):
        dist = 2.29*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (3.01*5280, 8.87*5280):
        dist = 2.96*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (3.29*5280, 9.23*5280):
        dist = 3.41*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (3.42*5280, 9.61*5280):
        dist = 3.82*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (7.45*5280, 3.86*5280):
        dist = 4.83*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (7.7*5280, 3.61*5280):
        dist = 5.19*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (8.04*5280, 3.28*5280):
        dist = 5.66*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (8.39*5280, 2.98*5280):
        dist = 6.12*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (8.75*5280, 2.73*5280):
        dist = 6.56*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (9.13*5280, 2.5*5280):
        dist = 7*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (9.53*5280, 2.29*5280):
        dist = 7.46*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (10.54*5280, 0.9*5280):
        dist = 9.93*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (3.1*5280, 7.24*5280)) or (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 4.78*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (2.77*5280, 7.57*5280)) or (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 4.31*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (2.43*5280, 7.91*5280)) or (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 3.83*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (2.61*5280, 8.34*5280)) or (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 3.36*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (3.01*5280, 8.87*5280)) or (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 2.7*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (3.29*5280, 9.23*5280)) or (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 2.24*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (3.42*5280, 9.61*5280)) or (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 1.84*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 10.49*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 10.85*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 11.32*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 11.78*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 12.22*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 12.66*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 13.11*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 15.59*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 5.10*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 4.75*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 4.27*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 3.81*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 3.38*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 2.93*5280
        return int(dist)
    if (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (10.54*5280, 0.9*5280)):
        dist = 2.48*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 5.71*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 6.53*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 7.49*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 8.41*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 9.52*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 10.42*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 11.27*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 6.06*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 6.54*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 7.00*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 7.43*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 7.88*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 8.33*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 6.18*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 7.01*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 7.47*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 7.91*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 8.35*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 8.80*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 6.66*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 7.02*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 7.95*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 8.39*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 8.83*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 9.28*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 7.13*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 7.48*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 7.96*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 8.85*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 9.30*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 9.75*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 7.79*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 8.14*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 8.62*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 9.08*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 9.96*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 10.41*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 8.25*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 8.60*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 9.08*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 9.53*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 9.97*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (9.53*5280, 2.29*5280)) or (stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 10.87*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (7.45*5280, 3.86*5280)) or (stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 8.64*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (7.7*5280, 3.61*5280)) or (stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 9.00*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (8.04*5280, 3.28*5280)) or (stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 9.48*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (8.39*5280, 2.98*5280)) or (stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 9.93*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (8.75*5280, 2.73*5280)) or (stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 10.37*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (9.13*5280, 2.5*5280)) or (stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 10.81*5280
        return int(dist)
    if stop_1 == (7.45*5280, 3.86*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 4.83*5280
        return int(dist)
    if stop_1 == (7.7*5280, 3.61*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 5.19*5280
        return int(dist)
    if stop_1 == (8.04*5280, 3.28*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 5.66*5280
        return int(dist)
    if stop_1 == (8.39*5280, 2.98*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 6.12*5280
        return int(dist)
    if stop_1 == (8.75*5280, 2.73*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 6.56*5280
        return int(dist)
    if stop_1 == (9.13*5280, 2.5*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 7.00*5280
        return int(dist)
    if stop_1 == (9.53*5280, 2.29*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 7.45*5280
        return int(dist)
    if stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 0.97*5280
        return int(dist)
    if stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 1.44*5280
        return int(dist)
    if stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 1.91*5280
        return int(dist)
    if stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 2.39*5280
        return int(dist)
    if stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 2.86*5280
        return int(dist)
    if stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 3.52*5280
        return int(dist)
    if stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 3.98*5280
        return int(dist)
    if stop_1 == (3.54*5280, 10.04*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 4.38*5280
        return int(dist)
    if stop_1 == (3.74*5280, 10.45*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 4.83*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 4.29*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (0.6*5280, 8.07*5280):
        dist = 4.29*5280
        return int(dist)
    if (stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.42*5280, 9.61*5280)):
        dist = 13.75*5280
        return int(dist)
    if (stop_1 == (3.74*5280, 10.45*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.74*5280, 10.45*5280)):
        dist = 14.65*5280
        return int(dist)
    if (stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.29*5280, 9.23*5280)):
        dist = 13.52*5280
        return int(dist)
    if (stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.01*5280, 8.87*5280)):
        dist = 12.89*5280
        return int(dist)
    if (stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (2.43*5280, 7.91*5280)):
        dist = 11.76*5280
        return int(dist)
    if (stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (2.61*5280, 8.34*5280)):
        dist = 12.22*5280
        return int(dist)
    if (stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (2.77*5280, 7.57*5280)):
        dist = 11.28*5280
        return int(dist)
    if (stop_1 == (3.54*5280, 10.04*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.54*5280, 10.04*5280)):
        dist = 14.19*5280
        return int(dist)
    if (stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (10.54*5280, 0.9*5280)) or (stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.1*5280, 7.24*5280)):
        dist = 10.81*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.54*5280, 10.04*5280):
        dist = 8.55*5280
        return int(dist)
    if stop_1 == (0.6*5280, 8.07*5280) and stop_2 == (3.74*5280, 10.45*5280):
        dist = 9.01*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (3.54*5280, 10.04*5280):
        dist = 8.55*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (3.74*5280, 10.45*5280):
        dist = 4.26*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (3.54*5280, 10.04*5280)) or (stop_1 == (3.54*5280, 10.04*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 1.39*5280
        return int(dist)
    if (stop_1 == (4.29*5280, 11.21*5280) and stop_2 == (3.74*5280, 10.45*5280)) or (stop_1 == (3.74*5280, 10.45*5280) and stop_2 == (4.29*5280, 11.21*5280)):
        dist = 0.94*5280
        return int(dist)
    if stop_1 == (10.54*5280, 0.9*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 9.89*5280
        return int(dist)
    if stop_1 == (10.44*5280, 0.96*5280) and stop_2 == (3.79*5280, 6.71*5280):
        dist = 9.77*5280
        return int(dist)
    if stop_1 == (3.79*5280, 6.71*5280) and stop_2 == (4.29*5280, 11.21*5280):
        dist = 5.75*5280
        return int(dist)
    if stop_1 == (3.74*5280, 10.45*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 11.61*5280
        return int(dist)
    if stop_1 == (3.54*5280, 10.04*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 11.51*5280
        return int(dist)
    if stop_1 == (3.42*5280, 9.61*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 10.21*5280
        return int(dist)
    if stop_1 == (3.29*5280, 9.23*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 9.80*5280
        return int(dist)
    if stop_1 == (3.01*5280, 8.87*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 9.37*5280
        return int(dist)
    if stop_1 == (2.61*5280, 8.34*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 8.74*5280
        return int(dist)
    if stop_1 == (2.43*5280, 7.91*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 8.28*5280
        return int(dist)
    if stop_1 == (2.77*5280, 7.57*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 8.06*5280
        return int(dist)
    if stop_1 == (3.1*5280, 7.24*5280) and stop_2 == (10.44*5280, 0.96*5280):
        dist = 7.87*5280
        return int(dist)


def euclidean_distance(position_1, position_2):  # This function calculates the distance UAV travels from one mission point to another
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])))


def print_solution(data, manager, routing, solution):  # Prints the UAV optimization solution output on the console.
    print("Objective: {}".format(solution.ObjectiveValue()))
    total_distance = 0
    total_fuel = 0
    total_time = 0
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
            previous_index = index
            index = solution.Value(routing.NextVar(index))
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
        fuel_remaining.append(solution.Value(fuel_var))
        plan_output += "Distance of the route: {} ft\n".format(solution.Value(dist_var))
        plan_output += "Remaining Fuel of the route: {}\n".format(data["fuel_capacity"] - solution.Value(fuel_var))
        plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
        print(plan_output)
        total_distance += solution.Value(dist_var)
        total_fuel += data['fuel_capacity'] - solution.Value(fuel_var)
        total_time += solution.Value(time_var)
    print('Total Distance of all routes: {} ft'.format(total_distance))
    print('Total Fuel remaining of all routes: {}'.format(total_fuel))
    print('Total Time of all routes: {} seconds'.format(total_time))
    return solution.ObjectiveValue(), total_distance, total_time


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


def main(params, f_eval={}):  # This is where optimization of UAV routes happen.
    """Entry point of the program."""
    # Instantiate the data problem.
    # [START data]
    try:
        ugv_stop_loc, NW_stop, SE_stop, strt_pt = params
        data, veh_max_time, depotb_vel, ugv_vel, pen, tw_dict, stop1, stop2 = create_data_model(ugv_stop_loc, NW_stop, SE_stop, strt_pt)
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
        # routing.SetArcCostEvaluatorOfAllVehicles(fuel_callback_index)
        routing.AddDimension(
            fuel_callback_index,
            data["fuel_capacity"],
            data["fuel_capacity"],
            True,
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
        routing.AddAtSolutionCallback(fuel_callback)
        # [END solve]

        # Print solution on console.
        # [START print_solution]
        # ugv_tw_array = np.array(data["time_windows"])
        if solution:
            obj_val, total_dist, total_time = print_solution(data, manager, routing, solution)
            route_dict, route = get_routes(solution, routing, manager)
            params = tuple(params)
            if obj_val < 1_000_000:
                f_eval[params] = veh_max_time - total_time
                print("Objective value is: {}".format(veh_max_time - total_time))
                return veh_max_time - total_time
            else:
                f_eval[params] = obj_val
                print("Objective value is: {}".format(obj_val))
                return obj_val
        else:
            print("*************No solution found************")
            return 0
    except Exception:
        pass
    # [END print_solution]
    # print("Solver status:", routing.status())
    # print("****************************************************\n")


# if __name__ == "__main__":
#     params = [58, 25, 10]
#     objval = main(params)
#     print(objval)
