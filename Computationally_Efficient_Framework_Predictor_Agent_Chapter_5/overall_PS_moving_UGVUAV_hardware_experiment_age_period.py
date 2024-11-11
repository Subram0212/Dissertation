import pandas as pd
import numpy as np
import minimum_set_cover_UGV_hyperparam_refined_replanning
import ugv_power_consumption
import ugv_power_consumption_subproblem1
import ugv_power_consumption_subproblem2
import sp1_age_period_hardware_exp as sp1
import sp2_age_period_hardware_exp as sp2
import sp3_age_period_hardware_exp as sp3
# import sp1_multiUGVstops_ateams_movingUGV_justend_rch_pts as sp1
# import sp2_multiUGV_stops_ateams_movingUGV_justend_rch_pts as sp2
# import sp3_multiUGV_stops_ateams_movingUGV_justend_rch_pts as sp3
import yaml
import ast
import random
import csv
import copy
import time
import matplotlib.pyplot as plt
import scipy
import ugv_distance_calc_auto
import math


# mission_points = pd.read_excel('ARL corridor reduced limited data points.xlsx', engine='openpyxl')
# ordered_pts_from_depot = pd.read_excel('ARL corridor reduced points ordered DepotB.xlsx', engine='openpyxl')
# ordered_pts_rest = pd.read_excel('ARL corridor reduced points ordered.xlsx', engine='openpyxl')
mission_points = pd.read_csv('Hardware experimental scaled up scenario in miles.csv')
mission_points = mission_points.values.tolist()
mission_points = [(round(m[0], 2), round(m[1], 2)) for m in mission_points]
ordered_pts_from_depot = pd.read_csv('Hardware experimental scaled up scenario in miles depotb ordered.csv')
ordered_pts_rest_df = pd.read_csv('Hardware experimental scaled up scenario in miles ordered.csv')
ordered_pts_from_depot = ordered_pts_from_depot.values.tolist()
ordered_pts_rest = ordered_pts_rest_df.values.tolist()
ordered_pts_from_depot = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot]
ordered_pts_rest = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_rest]
reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))


def ugv_mission_points(ugv_end_points, direction, ordered_pts_from_depot, ordered_pts_rest):
    ugv_locations = []
    rend_1 = ugv_end_points[0]
    rend_2 = ugv_end_points[1]
    count_1 = 0
    count_2 = 0
    if direction == 0:
        for i in range(len(ordered_pts_from_depot)):
            if ordered_pts_from_depot[i] != rend_1:
                count_1 += 1
            else:
                for j in range(count_1, len(ordered_pts_from_depot)):
                    if ordered_pts_from_depot[j] != rend_2:
                        ugv_locations.append(ordered_pts_from_depot[j])
                    else:
                        break
                break
    elif direction == 1:
        for i in range(len(ordered_pts_rest)):
            if ordered_pts_rest[i] != rend_1:
                count_2 += 1
            else:
                for j in range(count_2, len(ordered_pts_rest)):
                    if ordered_pts_rest[j] != rend_2:
                        ugv_locations.append(ordered_pts_rest[j])
                    else:
                        break
                break
    elif direction == 2:
        reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))
        for i in range(len(reversed_ordered_pts_rest)):
            if reversed_ordered_pts_rest[i] != rend_1:
                count_2 += 1
            else:
                for j in range(count_2, len(reversed_ordered_pts_rest)):
                    if reversed_ordered_pts_rest[j] != rend_2:
                        ugv_locations.append(reversed_ordered_pts_rest[j])
                    else:
                        break
                break
    ugv_locations.append(rend_2)
    return ugv_locations


def calculate_distance(point1, point2):
    # Calculate the Euclidean distance between two points
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def is_inside_radius(center, points, radius):
    # Initialize a list to store points inside the radius
    points_inside_radius = []

    # Iterate through the list of points
    for point in points:
        # Calculate the distance between the center and the current point
        distance = calculate_distance(center, point)

        # Check if the distance is less than or equal to the radius
        if distance <= radius:
            points_inside_radius.append(point)
    return points_inside_radius


def perch_and_move_agents_age_period(location, ugv_direction, sp1_end_pt, sp2_end_pt, sp3_end_pt, ugv_velocity):
    recharge_constant_time = 926
    uav_act_depart_location = []
    cumul_dist = 0
    cnt = 0
    ugv_visits_during_recharge = []
    if ugv_direction == 0:
        for i in ordered_pts_from_depot:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(ordered_pts_from_depot)):
                    if j == len(ordered_pts_from_depot)-1:
                        break
                    ugv_visits_during_recharge.append(ordered_pts_from_depot[j])
                    temp = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ordered_pts_from_depot[j], ordered_pts_from_depot[j+1]) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(ordered_pts_from_depot[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [ordered_pts_from_depot[-2]]
            if ordered_pts_from_depot.index(uav_act_depart_location[0]) > ordered_pts_from_depot.index(sp1_end_pt[0]):
                uav_act_depart_location = sp1_end_pt
            break
        if ordered_pts_from_depot.index(ugv_visits_during_recharge[-1]) > ordered_pts_from_depot.index(uav_act_depart_location[0]):
            diff = ordered_pts_from_depot.index(ugv_visits_during_recharge[-1]) - ordered_pts_from_depot.index(uav_act_depart_location[0])
            for i in range(diff):
                ugv_visits_during_recharge.pop()
    elif ugv_direction == 1:
        for i in ordered_pts_rest:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(ordered_pts_rest)):
                    if j == len(ordered_pts_rest)-1:
                        break
                    ugv_visits_during_recharge.append(ordered_pts_rest[j])
                    temp = ugv_distance_calc_auto.distance_btw_ugvstops(ordered_pts_rest[j], ordered_pts_rest[j+1]) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(ordered_pts_rest[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [ordered_pts_rest[-1]]
            if ordered_pts_rest.index(uav_act_depart_location[0]) > ordered_pts_rest.index(sp2_end_pt[0]):
                uav_act_depart_location = sp2_end_pt
            break
        if ordered_pts_rest.index(ugv_visits_during_recharge[-1]) > ordered_pts_rest.index(uav_act_depart_location[0]):
            diff = ordered_pts_rest.index(ugv_visits_during_recharge[-1]) - ordered_pts_rest.index(uav_act_depart_location[0])
            for i in range(diff):
                ugv_visits_during_recharge.pop()
    elif ugv_direction == 2:
        reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))
        for i in reversed_ordered_pts_rest:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(reversed_ordered_pts_rest)):
                    if j == len(reversed_ordered_pts_rest)-1:
                        break
                    ugv_visits_during_recharge.append(reversed_ordered_pts_rest[j])
                    temp = ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ordered_pts_rest[j], reversed_ordered_pts_rest[j+1]) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(reversed_ordered_pts_rest[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [reversed_ordered_pts_rest[-1]]
            if reversed_ordered_pts_rest.index(uav_act_depart_location[0]) > reversed_ordered_pts_rest.index(sp3_end_pt[0]):
                uav_act_depart_location = sp3_end_pt
            break
        if reversed_ordered_pts_rest.index(ugv_visits_during_recharge[-1]) > reversed_ordered_pts_rest.index(uav_act_depart_location[0]):
            diff = reversed_ordered_pts_rest.index(ugv_visits_during_recharge[-1]) - reversed_ordered_pts_rest.index(uav_act_depart_location[0])
            for i in range(diff):
                ugv_visits_during_recharge.pop()
    return uav_act_depart_location, ugv_visits_during_recharge


def perch_and_move_agents(location, sp1, sp2, sp3, sp1_end_pt, sp2_end_pt, sp3_end_pt, ugv_velocity):
    recharge_constant_time = 926
    uav_act_depart_location = []
    cumul_dist = 0
    cnt = 0
    if sp1 == 1:
        for i in ordered_pts_from_depot:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(ordered_pts_from_depot)):
                    if j == len(ordered_pts_from_depot)-1:
                        continue
                    temp = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ordered_pts_from_depot[j], ordered_pts_from_depot[j+1]) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(ordered_pts_from_depot[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [ordered_pts_from_depot[-2]]
            if ordered_pts_from_depot.index(uav_act_depart_location[0]) > ordered_pts_from_depot.index(sp1_end_pt[0]):
                uav_act_depart_location = sp1_end_pt
            break
    elif sp2 == 1:
        for i in ordered_pts_rest:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(ordered_pts_rest)):
                    if j == len(ordered_pts_rest)-1:
                        break
                    temp = ugv_distance_calc_auto.distance_btw_ugvstops(ordered_pts_rest[j], ordered_pts_rest[j+1]) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(ordered_pts_rest[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [ordered_pts_rest[-1]]
            if ordered_pts_rest.index(uav_act_depart_location[0]) > ordered_pts_rest.index(sp2_end_pt[0]):
                uav_act_depart_location = sp2_end_pt
            break
    elif sp3 == 1:
        reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))
        for i in reversed_ordered_pts_rest:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(reversed_ordered_pts_rest)):
                    if j == len(reversed_ordered_pts_rest)-1:
                        break
                    temp = ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ordered_pts_rest[j], reversed_ordered_pts_rest[j+1]) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(reversed_ordered_pts_rest[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [reversed_ordered_pts_rest[-1]]
            if reversed_ordered_pts_rest.index(uav_act_depart_location[0]) > reversed_ordered_pts_rest.index(sp3_end_pt[0]):
                uav_act_depart_location = sp3_end_pt
            break
    return uav_act_depart_location


def is_close(point1, point2, tolerance=1e-6):
    return abs(point1[0] - point2[0]) < tolerance and abs(point1[1] - point2[1]) < tolerance


def persistent_surveillance_replanning(strt_pt, ugv_power, param_list, num_uavs, ugv_stops_dict, visited_missions_so_far, ugv_direction, stp_depart_time, recharge_time, subprob_cnt, visited_missions_so_far_with_time, added_mission_pts, Age_period):
    """
    Perform replanning while optimizing routes
    :param strt_pt: the point from where the mission is starting
    :param ugv_power: the power level at that point, from which the persistent surveillance is carried until the ugv_max_capacity
    :param param_list: ugv parameter list which is under optimization process
    :param param_val: the destination location/mid-rendezvous location hyperparam
    :param num_uavs: number of UAVs in the process
    :param ugv_stops_dict:
    :param rem_locs: the remaining locations for which the routing is made
    :param ugv_direction: the direction UGV is about to move on
    :param stp_depart_time: the time of departure from that strt_pt from where the rest of optimization happens
    :param recharge_time: the recharge time that is to be taken into account before doing the optimization
    :return:
    """
    if len(added_mission_pts) > 0:
        if added_mission_pts[0] not in mission_points and added_mission_pts[1] not in mission_points:
            mission_points.extend(added_mission_pts)
    ordered_pts_from_depot_once = [i for i in ordered_pts_from_depot]
    ordered_pts_rest_once = [i for i in ordered_pts_rest]
    reversed_ordered_pts_rest_once = list(reversed(ordered_pts_rest_once))
    replanning_count = 0
    route_dict = None
    route_dict2 = None
    route_dict3 = None
    fitness_val = 0
    fitness_val2 = 0
    fitness_val3 = 0
    uav_time = 0
    uav_time2 = 0
    uav_time3 = 0
    nw_wait_time = 0
    se_wait_time = 0
    ugv_travel_time_tot = 0
    ugv_inter_locs = 0
    mission_strt_pt = [ordered_pts_from_depot[0]]
    TOTAL_MISSION_TIME = 200*60
    charge_to_load = 926

    """Fixed parameters"""
    ugv_max_capacity = 25_010_000  # in J
    ugv_velocity = 15  # in ft/s
    uav_speed = 33  # in ft/s

    """Hyperparameter optimization"""

    overall_route_list = []
    if param_list[0] > len(ugv_stops_dict):
        param_list[0] = len(ugv_stops_dict)
    if param_list[0] < 2:
        param_list[0] = 2
    if param_list[1] > 10:  # TODO: Deal with changing these values when initial parameter set values in A-Teams code are changed!!!
        param_list[1] = 10
    if param_list[1] < 2:
        param_list[1] = 2
    ugv_opt_stop1 = [ugv_stops_dict[param_list[0]][0]]
    ugv_opt_stop2 = [ugv_stops_dict[param_list[0]][1]]
    strt_pt_tuple = mission_strt_pt[0]
    depot_to_stp_len = 0
    btw_stps_len = 0
    count_se = 0
    for i in ordered_pts_from_depot:
        if i != ugv_opt_stop1[0]:
            depot_to_stp_len += 1
        else:
            break
    depot_to_stp_len -= 1
    for j in ordered_pts_rest:
        if j != ugv_opt_stop1[0]:
            count_se += 1
            continue
        else:
            for k in range(count_se+1, len(ordered_pts_rest)):
                if ordered_pts_rest[k] != ugv_opt_stop2[0]:
                    btw_stps_len += 1
                else:
                    break
    ugv_opt_stop1_tuple = ugv_opt_stop1[0]
    ugv_opt_stop2_tuple = ugv_opt_stop2[0]
    strt_pt_ft = tuple([i*5280 for i in strt_pt_tuple])
    ugv_opt_stop1_ft = tuple([i*5280 for i in ugv_opt_stop1_tuple])
    ugv_opt_stop2_ft = tuple([i*5280 for i in ugv_opt_stop2_tuple])
    uav_rendezvous_arrvl_location_sp = []
    penalty = 0
    stp_travel_time = ugv_distance_calc_auto.distance_from_depotB(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    travel_time_btw_stops = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
    ugv_power_from_nearend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp_travel_time)
    ugv_power_btw_stops = ((464.8*(ugv_velocity / 3.281) + 356.3)*travel_time_btw_stops)
    stp2_return_to_depot_time = sp1.euclidean_distance(strt_pt_ft, ugv_opt_stop2_ft) // ugv_velocity
    ugv_power_from_farend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp2_return_to_depot_time)
    ugv_power_direct_from_stop2 = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp2_return_to_depot_time)
    stp1_return_to_depot_time = sp1.euclidean_distance(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    ugv_power_direct_from_stop1 = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp1_return_to_depot_time)
    tot_uav_visits_sp1 = count_se - 1  # to get rid of the first point in ordered_list_rest
    reverse_pts = list(reversed(ordered_pts_rest))
    count_uav_points = 0
    for j in reverse_pts:
        if j != ugv_opt_stop2[0]:
            count_uav_points += 1
            continue
        else:
            break
    tot_uav_visits_sp2 = count_uav_points
    branch_mid_pt = [(3.4, 7.15)]
    additional_pts_for_sp3_cnt = 0
    for i in ordered_pts_from_depot:
        if i != branch_mid_pt[0]:
            additional_pts_for_sp3_cnt += 1
    tot_uav_visits_sp3 = (count_se - 1) + (additional_pts_for_sp3_cnt - 1)   # to not consider Depot B point
    track_node_age = {nodes: [0] for nodes in mission_points}
    # Age_period = {nodes: 1000 for nodes in mission_points}
    age_period_cnt = subprob_cnt
    uav_strt_time = stp_depart_time
    for node, time in visited_missions_so_far_with_time.items():
        track_node_age[node].append(time)
    route_overall_dict = []
    # Initially go from start to first UGV stop
    sp1_flag = 1
    visited_missions = []
    # visited_missions_so_far = []
    route_location_visits = []
    route_location_visits2 = []
    route_location_visits3 = []
    subprob_feasibility_list = []
    rend_cnt = 0
    ugv_wait_time = 0
    stp_reach_time = 0
    ugv_visits = []
    ugv_ordered_visits = []
    mission_time = 0
    subprob_cnt_list = []
    print(f"The UGV end point locations are: Stop 1 -> {ugv_opt_stop1} and Stop 2 -> {ugv_opt_stop2}")
    Age_period_for_obj = {node: [0] for node in mission_points}
    time_on_UGV_charging = 0
    ugv_visits_total_missions = []
    count_d_to_stp1 = 0
    for i, node in enumerate(ordered_pts_from_depot):
        if node != strt_pt[0]:
            count_d_to_stp1 += 1
        else:
            for j in range(count_d_to_stp1, len(ordered_pts_from_depot)):
                if ordered_pts_from_depot[j] != ugv_opt_stop1_tuple:
                    ugv_visits_total_missions.append(ordered_pts_from_depot[j])
                else:
                    break
    count_d_to_stp1 = 0
    for i, node in enumerate(ordered_pts_rest):
        if node != ugv_opt_stop1_tuple:
            count_d_to_stp1 += 1
        else:
            for j in range(count_d_to_stp1-1, len(ordered_pts_rest)):
                if ordered_pts_rest[j] != ugv_opt_stop2_tuple:
                    ugv_visits_total_missions.append(ordered_pts_rest[j])
                else:
                    break
    count_d_to_stp1 = 0
    while mission_time <= TOTAL_MISSION_TIME:
        if ugv_direction == 0:
            # uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[2], param_list[3]]*3
            uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
        elif ugv_direction == 1:
            # uav_rendezvous_arrvl_location_sp = [param_list[2]] + [param_list[3], param_list[1]]*3
            uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
        elif ugv_direction == 2:
            # uav_rendezvous_arrvl_location_sp = [param_list[3]] + [param_list[1], param_list[2]]*3
            uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
        if ugv_direction == 0:
            stop1 = False
            stop2 = False
            fleeting_location = strt_pt
            cnt = 0
            for i in range(cnt, len(ordered_pts_from_depot)):
                if ordered_pts_from_depot[i] != fleeting_location[0]:
                    cnt += 1
                else:
                    break
            uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
            if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_from_depot):
                destination_loc = ugv_opt_stop1
            else:
                destination_loc = [ordered_pts_from_depot[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
            if ordered_pts_from_depot.index(destination_loc[0]) > ordered_pts_from_depot.index(ugv_opt_stop1[0]):
                destination_loc = ugv_opt_stop1
        elif ugv_direction == 1:
            if replanning_count == 0:
                stop1 = True
                stop2 = False
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(ordered_pts_rest_once)):
                    if ordered_pts_rest_once[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_rest):
                    destination_loc = ugv_opt_stop2
                else:
                    destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if ordered_pts_rest_once.index(destination_loc[0]) > ordered_pts_rest_once.index(ugv_opt_stop2[0]):
                    destination_loc = ugv_opt_stop2
                replanning_count += 1
            else:
                stop1 = True
                stop2 = False
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(ordered_pts_rest)):
                    if ordered_pts_rest[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_rest):
                    destination_loc = ugv_opt_stop2
                else:
                    destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if ordered_pts_rest.index(destination_loc[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                    destination_loc = ugv_opt_stop2
        elif ugv_direction == 2:
            if replanning_count == 0:
                stop1 = False
                stop2 = True
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(reversed_ordered_pts_rest_once)):
                    if reversed_ordered_pts_rest_once[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(reversed_ordered_pts_rest):
                    destination_loc = ugv_opt_stop1
                else:
                    destination_loc = [reversed_ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if reversed_ordered_pts_rest_once.index(destination_loc[0]) > reversed_ordered_pts_rest_once.index(ugv_opt_stop1[0]):
                    destination_loc = ugv_opt_stop1
                replanning_count += 1
            else:
                stop1 = False
                stop2 = True
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(reversed_ordered_pts_rest)):
                    if reversed_ordered_pts_rest[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(reversed_ordered_pts_rest):
                    destination_loc = ugv_opt_stop1
                else:
                    destination_loc = [reversed_ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if reversed_ordered_pts_rest.index(destination_loc[0]) > reversed_ordered_pts_rest.index(ugv_opt_stop1[0]):
                    destination_loc = ugv_opt_stop1
        ugv_visits_dr_rhg = []
        while ugv_power <= ugv_max_capacity:
            itr_cnt = 0
            flag = 1
            ugv_power_check1 = (ugv_max_capacity - ugv_power_from_nearend_to_depot)
            ugv_power_check2 = (ugv_max_capacity - (ugv_power_btw_stops+(ugv_power_from_farend_to_depot)))
            ugv_power_check3 = ((ugv_power_from_nearend_to_depot + ugv_power_btw_stops))
            if ugv_direction == 1:
                if ugv_power <= ugv_power_check1 and ugv_power <= ugv_power_check2 and stop1 is True and ugv_power_check2 > 0:  # The condition says, if the ugv power consumption is less than the power that is remaining after it has consumed by traveling to respective locations, then continue in the condition.
                    flag = 1
                else:
                    flag = 0
            elif ugv_direction == 2:
                if ugv_max_capacity - ugv_power > ugv_power_check3 and stop2 is True and ugv_power_check3 > 0:
                    flag = 1
                else:
                    flag = 0
            if flag == 0:
                if stop1:
                    ugv_power += ugv_power_direct_from_stop1
                    uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop1_ft, strt_pt_ft) // uav_speed
                    break
                elif stop2:
                    ugv_power += ugv_power_direct_from_stop2
                    uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop2_ft, strt_pt_ft) // uav_speed
                    break
            ugv_visits = []
            if ugv_direction == 0:
                count_d_to_stp1 = 0
                for i, node in enumerate(ordered_pts_from_depot):
                    if node != strt_pt[0]:
                        count_d_to_stp1 += 1
                    else:
                        for j in range(count_d_to_stp1, len(ordered_pts_from_depot)):
                            if ordered_pts_from_depot[j] != ugv_opt_stop1_tuple:
                                ugv_visits.append(ordered_pts_from_depot[j])
                            else:
                                break
                ugv_visits_time_track = {}
                timet = stp_depart_time
                for i in range(len(ugv_visits)-1):
                    curr_node = tuple([j*5280 for j in ugv_visits[i]])
                    next_node = tuple([j*5280 for j in ugv_visits[i+1]])
                    each_stop_ugv_travel_T = sp1.euclidean_distance(curr_node, next_node) // ugv_velocity
                    timet += each_stop_ugv_travel_T
                    ugv_visits_time_track[ugv_visits[i+1]] = timet
                ugv_strt_pt = ugv_visits.pop(0)
                # print(f"UGV visits time track: {ugv_visits_time_track}")
            elif ugv_direction == 1:
                count_d_to_stp1 = 0
                for i, node in enumerate(ordered_pts_rest):
                    if node != ugv_opt_stop1_tuple:
                        count_d_to_stp1 += 1
                    else:
                        for j in range(count_d_to_stp1-1, len(ordered_pts_rest)):
                            if ordered_pts_rest[j] != ugv_opt_stop2_tuple:
                                ugv_visits.append(ordered_pts_rest[j])
                            else:
                                break
                ugv_visits_time_track = {}
                timet = stp_depart_time
                for i in range(len(ugv_visits)-1):
                    each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_btw_ugvstops(ugv_visits[i], ugv_visits[i+1]) // ugv_velocity
                    timet += each_stop_ugv_travel_T
                    ugv_visits_time_track[ugv_visits[i+1]] = timet
                ugv_visits.pop(0)
                ugv_strt_pt = ugv_visits[0]
                # print(f"UGV visits time track: {ugv_visits_time_track}")
            elif ugv_direction == 2:
                count_d_to_stp1 = 0
                for i, node in enumerate(reversed_ordered_pts_rest):
                    if node != ugv_opt_stop2_tuple:
                        count_d_to_stp1 += 1
                    else:
                        for j in range(count_d_to_stp1-1, len(reversed_ordered_pts_rest)):
                            if reversed_ordered_pts_rest[j] != ugv_opt_stop1_tuple:
                                ugv_visits.append(reversed_ordered_pts_rest[j])
                            else:
                                break
                ugv_visits_time_track = {}
                timet = stp_depart_time
                for i in range(len(ugv_visits)-1):
                    each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_btw_ugvstops(ugv_visits[i], ugv_visits[i+1]) // ugv_velocity
                    timet += each_stop_ugv_travel_T
                    ugv_visits_time_track[ugv_visits[i+1]] = timet
                ugv_visits.pop(0)
                ugv_strt_pt = ugv_visits[0]
                # print(f"UGV visits time track: {ugv_visits_time_track}")
            while flag:
                # ugv_temp_locs.insert(-1, destination_loc[0])
                # rchg_node_time = ugv_visits_time_track[route_locs_only[-1]]
                if age_period_cnt != 0:
                    if recharge_time == 0:
                        ugv_depart_time = stp_depart_time
                    elif recharge_time == 926:
                        ugv_depart_time = stp_depart_time + recharge_time
                    if len(ugv_visits_dr_rhg) >= 3:
                        ugv_depart_time = ugv_visits_time_track[ugv_visits_dr_rhg[-2]] + (ugv_visits_time_track[ugv_visits_dr_rhg[-2]] - ugv_visits_time_track[ugv_visits_dr_rhg[-3]])
                    elif len(ugv_visits_dr_rhg) == 2:
                        if ugv_visits_dr_rhg[-2] in ugv_visits_time_track.keys():
                            ugv_depart_time = ugv_visits_time_track[ugv_visits_dr_rhg[-2]]
                        else:
                            ugv_depart_time = stp_depart_time
                    else:
                        ugv_depart_time = stp_depart_time
                    for q, nde in Age_period.items():
                        Age_period[q] += time_on_UGV_charging
                        if q in ugv_visits_dr_rhg:
                            if q == ugv_visits_dr_rhg[-1]:
                                Age_period[q] = 0
                            elif q not in ugv_visits_time_track:
                                Age_period[q] = time_on_UGV_charging
                            else:
                                Age_period[q] = ugv_depart_time - ugv_visits_time_track[q]
                if ugv_direction == 0:
                    veh_max_time = uav_strt_time + ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(fleeting_location[0], destination_loc[0]) // ugv_velocity
                    fitness_val, mission_time, uav_time, total_distance, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp1.main(fleeting_location[0], ugv_opt_stop1, destination_loc, stp_depart_time, visited_missions, num_uavs, added_mission_pts, Age_period)
                    if age_period_cnt == 0:
                        Age_period = {nodes: 0 for nodes in mission_points}
                        age_period_cnt += 1
                elif ugv_direction == 1:
                    veh_max_time = uav_strt_time + ugv_distance_calc_auto.distance_btw_ugvstops(fleeting_location[0], destination_loc[0]) // ugv_velocity
                    fitness_val, mission_time, uav_time, total_distance, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp2.main(fleeting_location, ugv_opt_stop2, destination_loc, stp_depart_time, recharge_time, visited_missions, num_uavs, added_mission_pts, Age_period)
                    if age_period_cnt == 0:
                        Age_period = {nodes: 0 for nodes in mission_points}
                        age_period_cnt += 1
                elif ugv_direction == 2:
                    veh_max_time = uav_strt_time + ugv_distance_calc_auto.distance_btw_ugvstops(fleeting_location[0], destination_loc[0]) // ugv_velocity
                    fitness_val, mission_time, uav_time, total_distance, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp3.main(fleeting_location, ugv_opt_stop1, destination_loc, stp_depart_time, recharge_time, visited_missions, num_uavs, added_mission_pts, Age_period)
                    if age_period_cnt == 0:
                        Age_period = {nodes: 0 for nodes in mission_points}
                        age_period_cnt += 1
                time_on_UGV_charging = 926
                if fitness_val >= 1_000_000:
                    subprob_feasibility_list.append(0)
                else:
                    subprob_feasibility_list.append(1)
                if fitness_val == 0 or len(route_dict) == 2:
                    stp_depart_time = tw_dict[12][0] + time_on_UGV_charging
                    uav_time = tw_dict[12][0] + time_on_UGV_charging
                else:
                    stp_depart_time = uav_time
                if route_dict[-2][0] == ugv_opt_stop1_tuple or route_dict[-2][0] == ugv_opt_stop2_tuple or route_dict[-2][0] == destination_loc or fitness_val == 0:
                    recharge_time = 0 + 1800
                else:
                    recharge_time = time_on_UGV_charging + 1800
                overall_route_list.append(route_dict)
                route_locs_only = [i[0] for i in route_dict]
                for l in route_dict:
                    # if l[0] == fleeting_location[0] or l[0] == destination_loc[0] or l[0] == ugv_opt_stop1[0]:
                    #     continue
                    visited_missions.append(l[0])
                    # visited_missions_so_far.append(l[0])
                ugv_temp_locs = ugv_mission_points((fleeting_location[0], destination_loc[0]), ugv_direction, ordered_pts_from_depot, ordered_pts_rest)
                for i in ugv_temp_locs:
                    visited_missions_so_far.append(i)
                for i in route_locs_only:
                    visited_missions_so_far.append(i)
                for i in ugv_visits_dr_rhg:
                    visited_missions_so_far.append(i)
                for q in Age_period.keys():
                    if q in route_locs_only:
                        # Calculate age for points in the current route
                        idx_num = route_locs_only.index(q)
                        time_reference = uav_strt_time if q == ugv_strt_pt else route_dict[idx_num][1]
                        if q in ugv_visits and ugv_direction == 0:
                            time_reference = max(ugv_visits_time_track[q], route_dict[idx_num][1]) if ordered_pts_from_depot.index(q) < ordered_pts_from_depot.index(destination_loc[0]) else route_dict[idx_num][1]
                        Age_period[q] = uav_time - time_reference
                    else:
                        # Calculate age for points not in the current route
                        Age_period[q] = uav_time - (uav_strt_time if q in visited_missions_so_far else 0)
                print(f"Age_period: {Age_period}")
                if uav_time > veh_max_time or (veh_max_time - uav_time < 60) or (uav_time - veh_max_time < 60):
                    uav_strt_time = uav_time + recharge_time
                else:
                    uav_strt_time = veh_max_time + recharge_time
                total_ugv_uav_visits = ugv_visits+visited_missions_so_far
                total_ugv_uav_visits = [x for i, x in enumerate(total_ugv_uav_visits) if x not in total_ugv_uav_visits[:i]]
                total_ugv_uav_visits = {i: 0 for i in total_ugv_uav_visits}
                for ley, val in ugv_visits_time_track.items():
                    total_ugv_uav_visits[ley] = val
                for i in overall_route_list:
                    for j in i:
                        total_ugv_uav_visits[j[0]] = j[1]
                if destination_loc[0] == ugv_opt_stop1[0] or destination_loc[0] == ugv_opt_stop2[0]:
                    itr_cnt += 1
                    covered_points1 = is_inside_radius(ugv_opt_stop1[0], mission_points, 24750/5280)
                    covered_points2 = is_inside_radius(ugv_opt_stop2[0], mission_points, 24750/5280)
                    intersection = [element for element in covered_points1 if element in covered_points2]
                    covered_points = []
                    if destination_loc[0] == ugv_opt_stop1[0]:
                        covered_points = [element for element in covered_points1 if element not in intersection]
                    elif destination_loc[0] == ugv_opt_stop2[0]:
                        covered_points = [element for element in covered_points2 if element not in intersection]
                    missing_elements = [element for element in covered_points if not any(is_close(element, mission) for mission in visited_missions)]
                    missing_elements = [element for element in missing_elements if element not in ugv_visits_total_missions]
                    print(f"Missing elements: {missing_elements} and ugv_visits: {ugv_visits_total_missions}")
                    if (len(missing_elements) > 0 or fitness_val == 0) and itr_cnt < 3:
                        fleeting_location = destination_loc
                        continue
                    if ugv_direction == 0:
                        ugv_direction = 1
                        stp_travel_time = ugv_distance_calc_auto.distance_from_depotB(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
                        stp_reach_time = stp_travel_time
                        if fitness_val == 0:
                            ugv_wait_time = 0
                        else:
                            ugv_wait_time = route_dict[-1][1] - tw_dict[12][0]
                        ugv_power += ugv_power_consumption_subproblem1.ugv_power(ugv_wait_time, ugv_velocity / 3.281, stp_reach_time)
                        rend_cnt += 1
                    elif ugv_direction == 1:
                        ugv_direction = 2
                        stp_travel_time = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
                        stp_reach_time = stp_travel_time
                        if fitness_val == 0:
                            ugv_wait_time = 0
                        else:
                            ugv_wait_time = route_dict[-1][1] - tw_dict[12][0]
                        ugv_power += ugv_power_consumption_subproblem1.ugv_power(ugv_wait_time, ugv_velocity / 3.281, stp_reach_time)
                        rend_cnt += 1
                    elif ugv_direction == 2:
                        ugv_direction = 1
                        stp_travel_time = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
                        stp_reach_time = stp_travel_time
                        if fitness_val == 0:
                            ugv_wait_time = 0
                        else:
                            ugv_wait_time = route_dict[-1][1] - tw_dict[12][0]
                        ugv_power += ugv_power_consumption_subproblem1.ugv_power(ugv_wait_time, ugv_velocity / 3.281, stp_reach_time)
                        rend_cnt += 1
                    flag = 0
                fleeting_location, ugv_visits_dr_rhg = perch_and_move_agents_age_period(destination_loc, ugv_direction, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
                if ugv_direction == 0:
                    cnt = 0
                    for i in range(cnt, len(ordered_pts_from_depot)):
                        if ordered_pts_from_depot[i] != fleeting_location[0]:
                            cnt += 1
                        else:
                            break
                    uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                    if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_from_depot):
                        destination_loc = ugv_opt_stop1
                    else:
                        destination_loc = [ordered_pts_from_depot[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                    if ordered_pts_from_depot.index(destination_loc[0]) > ordered_pts_from_depot.index(ugv_opt_stop1[0]):
                        destination_loc = ugv_opt_stop1
                elif ugv_direction == 1:
                    cnt = 0
                    for i in range(cnt, len(ordered_pts_rest)):
                        if ordered_pts_rest[i] != fleeting_location[0]:
                            cnt += 1
                        else:
                            break
                    uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                    if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_rest):
                        destination_loc = ugv_opt_stop2
                    else:
                        destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                    if ordered_pts_rest.index(destination_loc[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                        destination_loc = ugv_opt_stop2
                    stop1 = True
                    stop2 = False
                elif ugv_direction == 2:
                    cnt = 0
                    for i in range(cnt, len(reversed_ordered_pts_rest)):
                        if reversed_ordered_pts_rest[i] != fleeting_location[0]:
                            cnt += 1
                        else:
                            break
                    uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                    if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(reversed_ordered_pts_rest):
                        destination_loc = ugv_opt_stop1
                    else:
                        destination_loc = [reversed_ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                    if reversed_ordered_pts_rest.index(destination_loc[0]) > reversed_ordered_pts_rest.index(ugv_opt_stop1[0]):
                        destination_loc = ugv_opt_stop1
                    stop1 = False
                    stop2 = True
                for m in route_dict:
                    route_location_visits.append(m)
                visited_missions = []
            for node, age_period in Age_period.items():
                Age_period_for_obj[node].append(age_period)
            subprob_cnt += 1
            route_location_visits.sort(key=lambda x: x[1])
            route_overall_dict.append(route_location_visits)
            # if fitness_val > 1_000_000 and len(visited_missions) != tot_uav_visits_sp1:
            #     penalty += 500_000_000
            # each_stop_ugv_travel_T = stp1_reach_time // count_d_to_stp1
            ugv_ordered_visits = []

            visited_missions_so_far.extend(ugv_visits)
            if len(ugv_visits) > 0:
                ugv_visits.pop()

            # stp1_depart_time = tw_dict[7][1]
            stop1 = True

            route_locations = []
            for k, node in enumerate(route_location_visits):
                route_locations.append(route_location_visits[k][0])

            for n, node in enumerate(route_location_visits):
                if node == ((0, 0), 0):
                    continue
                track_node_age[node[0]].append(route_location_visits[n][1])
            for m, node in enumerate(ugv_visits):
                track_node_age[node].append(ugv_visits_time_track[node])
        for node, age in track_node_age.items():
            track_node_age[node] = list(sorted(set(age)))
        if fitness_val == 0:
            uav_time = stp_depart_time
            penalty += 10000
        for q, nde in Age_period.items():
            if stop1 is True:
                Age_period[q] = Age_period[q] + stp1_return_to_depot_time
            else:
                Age_period[q] = Age_period[q] + stp2_return_to_depot_time
        print("*****************************************************************")
        print(f'The UGV power consumption throughout the mission is: {ugv_power}')
        print("*****************************************************************")
        if stop1 is True:
            stp_depart_time += stp1_return_to_depot_time
        else:
            stp_depart_time += stp2_return_to_depot_time
        fleeting_location = mission_strt_pt
        ugv_direction = 0
        stp_depart_time += 300
        if subprob_cnt <= 1:
            mission_time = stp_depart_time +60000
        else:
            mission_time = stp_depart_time
        ugv_power = 0
        recharge_time = 0
        subprob_cnt_list.append(subprob_cnt)
        subprob_cnt = 0
    for node, age in track_node_age.items():
        track_node_age[node].append(uav_time)
    mission_end_time = uav_time  # this is for the case with UGV flexibility of traveling directly to end point instead of route
    print(f"The mission of Persistent surveillance (Mission 1) ends at: {mission_end_time} seconds")
    # mission_end_time = timet + each_stop_ugv_travel_T
    if ugv_power > ugv_max_capacity:
        penalty += 500_000_000
    rem_fuel = ugv_max_capacity - ugv_power
    # if subprob_cnt < 3:
    #     penalty += 10_000_000

    """Calculation of the penalty cost - aka the age period of the nodes"""
    mission_cost = {}
    for key, val in track_node_age.items():
        cost_val = 0
        if len(val) == 0:
            continue
        elif len(val) == 1:
            for i in range(len(val)):
                # cost_val += scipy.integrate.quad(integral_function, 0, val[i], args=0)[0]
                cost_val += ((mission_end_time//60 - val[i]//60)**2 / 3600)
        else:
            for i in range(len(val)-1):
                # cost_val += scipy.integrate.quad(integral_function, val[i], val[i+1], args=val[i])[0]
                cost_val += ((val[i+1]//60 - val[i]//60)**2 / 3600)
        mission_cost[key] = cost_val

    # Calculate the total cost
    total_cost = 0
    for key1, val1 in mission_cost.items():
        total_cost += val1
    print(f'Age of the nodes: {track_node_age}')
    # print(f"\033[93m Total cost: {total_cost} \033[0m")
    for key, val in track_node_age.items():
        if len(val) < 3:
            if key in ugv_visits:
                penalty += 0
            else:
                penalty += 1_000_000
    max_of_uav_ugv_travel = max(subprob_cnt_list)
    total_cost -= 100 * np.log(max_of_uav_ugv_travel)
    print(f'The total score accquired during this mission is: {total_cost+penalty}')
    return mission_end_time - uav_time, mission_end_time, ugv_velocity, ugv_velocity, overall_route_list, route_dict, route_dict2, route_dict3, subprob_feasibility_list, ugv_ordered_visits, total_cost+penalty, visited_missions_so_far, ugv_power, fleeting_location, destination_loc


def persistent_surveillance_replanning_nelder_mead(param_list, strt_pt, ugv_power, num_uavs, ugv_stops_dict, visited_missions_so_far, ugv_direction, stp_depart_time, recharge_time, subprob_cnt, visited_missions_so_far_with_time, Age_period, added_mission_pts, feval={}):
    """
    Perform replanning while optimizing routes
    :param strt_pt: the point from where the mission is starting
    :param ugv_power: the power level at that point, from which the persistent surveillance is carried until the ugv_max_capacity
    :param param_list: ugv parameter list which is under optimization process
    :param param_val: the destination location/mid-rendezvous location hyperparam
    :param num_uavs: number of UAVs in the process
    :param ugv_stops_dict:
    :param rem_locs: the remaining locations for which the routing is made
    :param ugv_direction: the direction UGV is about to move on
    :param stp_depart_time: the time of departure from that strt_pt from where the rest of optimization happens
    :param recharge_time: the recharge time that is to be taken into account before doing the optimization
    :return:
    """
    if len(added_mission_pts) > 0:
        if added_mission_pts[0] not in mission_points and added_mission_pts[1] not in mission_points:
            mission_points.extend(added_mission_pts)
    ordered_pts_from_depot_once = [i for i in ordered_pts_from_depot]
    ordered_pts_rest_once = [i for i in ordered_pts_rest]
    reversed_ordered_pts_rest_once = list(reversed(ordered_pts_rest_once))
    replanning_count = 0
    route_dict = None
    route_dict2 = None
    route_dict3 = None
    fitness_val = 0
    fitness_val2 = 0
    fitness_val3 = 0
    uav_time = 0
    uav_time2 = 0
    uav_time3 = 0
    nw_wait_time = 0
    se_wait_time = 0
    ugv_travel_time_tot = 0
    ugv_inter_locs = 0
    mission_strt_pt = [ordered_pts_from_depot[0]]
    TOTAL_MISSION_TIME = 200*60
    charge_to_load = 926

    """Fixed parameters"""
    ugv_max_capacity = 25_010_000  # in J
    ugv_velocity = 15  # in ft/s
    uav_speed = 33  # in ft/s

    """Changing finite states"""
    sp1_flag = 0
    sp2_flag = 0
    sp3_flag = 0

    """Hyperparameter optimization"""

    ugv_travel_time = 0
    stp2_depart_time = 0
    # stp1_depart_time = 0
    uav_time2, uav_time3, uav_time4 = 0, 0, 0
    overall_route_list = []
    # strt_pt = [ordered_pts_from_depot[0]]
    param_list = [int(i) for i in param_list]
    if param_list[0] > len(ugv_stops_dict):
        param_list[0] = len(ugv_stops_dict)
    if param_list[0] < 2:
        param_list[0] = 2
    if param_list[1] > 10:  # TODO: Deal with changing these values when initial parameter set values in A-Teams code are changed!!!
        param_list[1] = 10
    if param_list[1] < 2:
        param_list[1] = 2
    # if param_list[5] > 11:
    #     param_list[5] = 11
    # if param_list[5] < 2:
    #     param_list[5] = 2
    ugv_opt_stop1 = [ugv_stops_dict[param_list[0]][0]]
    ugv_opt_stop2 = [ugv_stops_dict[param_list[0]][1]]
    strt_pt_tuple = mission_strt_pt[0]
    depot_to_stp_len = 0
    btw_stps_len = 0
    count_se = 0
    for i in ordered_pts_from_depot:
        if i != ugv_opt_stop1[0]:
            depot_to_stp_len += 1
        else:
            break
    depot_to_stp_len -= 1
    for j in ordered_pts_rest:
        if j != ugv_opt_stop1[0]:
            count_se += 1
            continue
        else:
            for k in range(count_se+1, len(ordered_pts_rest)):
                if ordered_pts_rest[k] != ugv_opt_stop2[0]:
                    btw_stps_len += 1
                else:
                    break
    ugv_opt_stop1_tuple = ugv_opt_stop1[0]
    ugv_opt_stop2_tuple = ugv_opt_stop2[0]
    strt_pt_ft = tuple([i*5280 for i in strt_pt_tuple])
    ugv_opt_stop1_ft = tuple([i*5280 for i in ugv_opt_stop1_tuple])
    ugv_opt_stop2_ft = tuple([i*5280 for i in ugv_opt_stop2_tuple])
    uav_rendezvous_arrvl_location_sp = []
    if ugv_direction == 0:
        # uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[2], param_list[3]]*3
        uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
    elif ugv_direction == 1:
        # uav_rendezvous_arrvl_location_sp = [param_list[2]] + [param_list[3], param_list[1]]*3
        uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
    elif ugv_direction == 2:
        # uav_rendezvous_arrvl_location_sp = [param_list[3]] + [param_list[1], param_list[2]]*3
        uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
    penalty = 0
    stp_travel_time = ugv_distance_calc_auto.distance_from_depotB(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    travel_time_btw_stops = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
    ugv_power_from_nearend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp_travel_time)
    ugv_power_btw_stops = ((464.8*(ugv_velocity / 3.281) + 356.3)*travel_time_btw_stops)
    stp2_return_to_depot_time = sp1.euclidean_distance(strt_pt_ft, ugv_opt_stop2_ft) // ugv_velocity
    ugv_power_from_farend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp2_return_to_depot_time)
    ugv_power_direct_from_stop2 = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp2_return_to_depot_time)
    stp1_return_to_depot_time = sp1.euclidean_distance(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    ugv_power_direct_from_stop1 = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp1_return_to_depot_time)
    tot_uav_visits_sp1 = count_se - 1  # to get rid of the first point in ordered_list_rest
    reverse_pts = list(reversed(ordered_pts_rest))
    count_uav_points = 0
    for j in reverse_pts:
        if j != ugv_opt_stop2[0]:
            count_uav_points += 1
            continue
        else:
            break
    tot_uav_visits_sp2 = count_uav_points
    branch_mid_pt = [(3.4, 7.15)]
    additional_pts_for_sp3_cnt = 0
    for i in ordered_pts_from_depot:
        if i != branch_mid_pt[0]:
            additional_pts_for_sp3_cnt += 1
    tot_uav_visits_sp3 = (count_se - 1) + (additional_pts_for_sp3_cnt - 1)   # to not consider Depot B point
    track_node_age = {nodes: [0] for nodes in mission_points}
    # Age_period = {nodes: 1000 for nodes in mission_points}
    age_period_cnt = subprob_cnt
    uav_strt_time = stp_depart_time
    for node, time in visited_missions_so_far_with_time.items():
        track_node_age[node].append(time)
    route_overall_dict = []
    # Initially go from start to first UGV stop
    sp1_flag = 1
    visited_missions = []
    # visited_missions_so_far = []
    route_location_visits = []
    route_location_visits2 = []
    route_location_visits3 = []
    subprob_feasibility_list = []
    rend_cnt = 0
    ugv_wait_time = 0
    stp_reach_time = 0
    ugv_visits = []
    ugv_ordered_visits = []
    mission_time = 0
    subprob_cnt_list = []
    time_on_UGV_charging = 0
    print(f"The UGV end point locations are: Stop 1 -> {ugv_opt_stop1} and Stop 2 -> {ugv_opt_stop2}")
    Age_period_for_obj = {node: [0] for node in mission_points}
    ugv_visits_total_missions = []
    count_d_to_stp1 = 0
    for i, node in enumerate(ordered_pts_from_depot):
        if node != strt_pt[0]:
            count_d_to_stp1 += 1
        else:
            for j in range(count_d_to_stp1, len(ordered_pts_from_depot)):
                if ordered_pts_from_depot[j] != ugv_opt_stop1_tuple:
                    ugv_visits_total_missions.append(ordered_pts_from_depot[j])
                else:
                    break
    count_d_to_stp1 = 0
    for i, node in enumerate(ordered_pts_rest):
        if node != ugv_opt_stop1_tuple:
            count_d_to_stp1 += 1
        else:
            for j in range(count_d_to_stp1-1, len(ordered_pts_rest)):
                if ordered_pts_rest[j] != ugv_opt_stop2_tuple:
                    ugv_visits_total_missions.append(ordered_pts_rest[j])
                else:
                    break
    count_d_to_stp1 = 0
    while mission_time <= TOTAL_MISSION_TIME:
        if ugv_direction == 0:
            # uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[2], param_list[3]]*3
            uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
        elif ugv_direction == 1:
            # uav_rendezvous_arrvl_location_sp = [param_list[2]] + [param_list[3], param_list[1]]*3
            uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
        elif ugv_direction == 2:
            # uav_rendezvous_arrvl_location_sp = [param_list[3]] + [param_list[1], param_list[2]]*3
            uav_rendezvous_arrvl_location_sp = [param_list[1]] + [param_list[1], param_list[1]]*15
        if ugv_direction == 0:
            stop1 = False
            stop2 = False
            fleeting_location = strt_pt
            cnt = 0
            for i in range(cnt, len(ordered_pts_from_depot)):
                if ordered_pts_from_depot[i] != fleeting_location[0]:
                    cnt += 1
                else:
                    break
            uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
            if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_from_depot):
                destination_loc = ugv_opt_stop1
            else:
                destination_loc = [ordered_pts_from_depot[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
            if ordered_pts_from_depot.index(destination_loc[0]) > ordered_pts_from_depot.index(ugv_opt_stop1[0]):
                destination_loc = ugv_opt_stop1
        elif ugv_direction == 1:
            if replanning_count == 0:
                stop1 = True
                stop2 = False
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(ordered_pts_rest_once)):
                    if ordered_pts_rest_once[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_rest):
                    destination_loc = ugv_opt_stop2
                else:
                    destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if ordered_pts_rest_once.index(destination_loc[0]) > ordered_pts_rest_once.index(ugv_opt_stop2[0]):
                    destination_loc = ugv_opt_stop2
                replanning_count += 1
            else:
                stop1 = True
                stop2 = False
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(ordered_pts_rest)):
                    if ordered_pts_rest[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_rest):
                    destination_loc = ugv_opt_stop2
                else:
                    destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if ordered_pts_rest.index(destination_loc[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                    destination_loc = ugv_opt_stop2
        elif ugv_direction == 2:
            if replanning_count == 0:
                stop1 = False
                stop2 = True
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(reversed_ordered_pts_rest_once)):
                    if reversed_ordered_pts_rest_once[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(reversed_ordered_pts_rest):
                    destination_loc = ugv_opt_stop1
                else:
                    destination_loc = [reversed_ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if reversed_ordered_pts_rest_once.index(destination_loc[0]) > reversed_ordered_pts_rest_once.index(ugv_opt_stop1[0]):
                    destination_loc = ugv_opt_stop1
                replanning_count += 1
            else:
                stop1 = False
                stop2 = True
                fleeting_location = strt_pt
                cnt = 0
                for i in range(cnt, len(reversed_ordered_pts_rest)):
                    if reversed_ordered_pts_rest[i] != fleeting_location[0]:
                        cnt += 1
                    else:
                        break
                uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(reversed_ordered_pts_rest):
                    destination_loc = ugv_opt_stop1
                else:
                    destination_loc = [reversed_ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                if reversed_ordered_pts_rest.index(destination_loc[0]) > reversed_ordered_pts_rest.index(ugv_opt_stop1[0]):
                    destination_loc = ugv_opt_stop1
        ugv_visits_dr_rhg = []
        while ugv_power <= ugv_max_capacity:
            itr_cnt = 0
            flag = 1
            ugv_power_check1 = (ugv_max_capacity - ugv_power_from_nearend_to_depot)
            ugv_power_check2 = (ugv_max_capacity - (ugv_power_btw_stops+(ugv_power_from_farend_to_depot)))
            ugv_power_check3 = ((ugv_power_from_nearend_to_depot + ugv_power_btw_stops))
            if ugv_direction == 1:
                if ugv_power <= ugv_power_check1 and ugv_power <= ugv_power_check2 and stop1 is True and ugv_power_check2 > 0:  # The condition says, if the ugv power consumption is less than the power that is remaining after it has consumed by traveling to respective locations, then continue in the condition.
                    flag = 1
                else:
                    flag = 0
            elif ugv_direction == 2:
                if ugv_max_capacity - ugv_power > ugv_power_check3 and stop2 is True and ugv_power_check3 > 0:
                    flag = 1
                else:
                    flag = 0
            if flag == 0:
                if stop1:
                    ugv_power += ugv_power_direct_from_stop1
                    uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop1_ft, strt_pt_ft) // uav_speed
                    break
                elif stop2:
                    ugv_power += ugv_power_direct_from_stop2
                    uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop2_ft, strt_pt_ft) // uav_speed
                    break
            ugv_visits = []
            if ugv_direction == 0:
                count_d_to_stp1 = 0
                for i, node in enumerate(ordered_pts_from_depot):
                    if node != strt_pt[0]:
                        count_d_to_stp1 += 1
                    else:
                        for j in range(count_d_to_stp1, len(ordered_pts_from_depot)):
                            if ordered_pts_from_depot[j] != ugv_opt_stop1_tuple:
                                ugv_visits.append(ordered_pts_from_depot[j])
                            else:
                                break
                ugv_visits_time_track = {}
                timet = stp_depart_time
                for i in range(len(ugv_visits)-1):
                    curr_node = tuple([j*5280 for j in ugv_visits[i]])
                    next_node = tuple([j*5280 for j in ugv_visits[i+1]])
                    each_stop_ugv_travel_T = sp1.euclidean_distance(curr_node, next_node) // ugv_velocity
                    timet += each_stop_ugv_travel_T
                    ugv_visits_time_track[ugv_visits[i+1]] = timet
                ugv_strt_pt = ugv_visits.pop(0)
                # print(f"UGV visits time track: {ugv_visits_time_track}")
            elif ugv_direction == 1:
                count_d_to_stp1 = 0
                for i, node in enumerate(ordered_pts_rest):
                    if node != ugv_opt_stop1_tuple:
                        count_d_to_stp1 += 1
                    else:
                        for j in range(count_d_to_stp1-1, len(ordered_pts_rest)):
                            if ordered_pts_rest[j] != ugv_opt_stop2_tuple:
                                ugv_visits.append(ordered_pts_rest[j])
                            else:
                                break
                ugv_visits_time_track = {}
                timet = stp_depart_time
                for i in range(len(ugv_visits)-1):
                    each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_btw_ugvstops(ugv_visits[i], ugv_visits[i+1]) // ugv_velocity
                    timet += each_stop_ugv_travel_T
                    ugv_visits_time_track[ugv_visits[i+1]] = timet
                ugv_visits.pop(0)
                ugv_strt_pt = ugv_visits[0]
                # print(f"UGV visits time track: {ugv_visits_time_track}")
            elif ugv_direction == 2:
                count_d_to_stp1 = 0
                for i, node in enumerate(reversed_ordered_pts_rest):
                    if node != ugv_opt_stop2_tuple:
                        count_d_to_stp1 += 1
                    else:
                        for j in range(count_d_to_stp1-1, len(reversed_ordered_pts_rest)):
                            if reversed_ordered_pts_rest[j] != ugv_opt_stop1_tuple:
                                ugv_visits.append(reversed_ordered_pts_rest[j])
                            else:
                                break
                ugv_visits_time_track = {}
                timet = stp_depart_time
                for i in range(len(ugv_visits)-1):
                    each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_btw_ugvstops(ugv_visits[i], ugv_visits[i+1]) // ugv_velocity
                    timet += each_stop_ugv_travel_T
                    ugv_visits_time_track[ugv_visits[i+1]] = timet
                ugv_visits.pop(0)
                ugv_strt_pt = ugv_visits[0]
                # print(f"UGV visits time track: {ugv_visits_time_track}")
            while flag:
                # ugv_temp_locs.insert(-1, destination_loc[0])
                # rchg_node_time = ugv_visits_time_track[route_locs_only[-1]]
                if age_period_cnt != 0:
                    if recharge_time == 0:
                        ugv_depart_time = stp_depart_time
                    elif recharge_time == 926:
                        ugv_depart_time = stp_depart_time + recharge_time
                    if len(ugv_visits_dr_rhg) >= 3:
                        ugv_depart_time = ugv_visits_time_track[ugv_visits_dr_rhg[-2]] + (ugv_visits_time_track[ugv_visits_dr_rhg[-2]] - ugv_visits_time_track[ugv_visits_dr_rhg[-3]])
                    elif len(ugv_visits_dr_rhg) == 2:
                        if ugv_visits_dr_rhg[-2] in ugv_visits_time_track.keys():
                            ugv_depart_time = ugv_visits_time_track[ugv_visits_dr_rhg[-2]]
                        else:
                            ugv_depart_time = stp_depart_time
                    else:
                        ugv_depart_time = stp_depart_time
                    for q, nde in Age_period.items():
                        Age_period[q] += time_on_UGV_charging
                        if q in ugv_visits_dr_rhg:
                            if q == ugv_visits_dr_rhg[-1]:
                                Age_period[q] = 0
                            elif q not in ugv_visits_time_track:
                                Age_period[q] = time_on_UGV_charging
                            else:
                                Age_period[q] = ugv_depart_time - ugv_visits_time_track[q]
                if ugv_direction == 0:
                    veh_max_time = uav_strt_time + ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(fleeting_location[0], destination_loc[0]) // ugv_velocity
                    fitness_val, mission_time, uav_time, total_distance, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp1.main(fleeting_location[0], ugv_opt_stop1, destination_loc, stp_depart_time, visited_missions, num_uavs, added_mission_pts, Age_period)
                    if age_period_cnt == 0:
                        Age_period = {nodes: 0 for nodes in mission_points}
                        age_period_cnt += 1
                elif ugv_direction == 1:
                    veh_max_time = uav_strt_time + ugv_distance_calc_auto.distance_btw_ugvstops(fleeting_location[0], destination_loc[0]) // ugv_velocity
                    fitness_val, mission_time, uav_time, total_distance, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp2.main(fleeting_location, ugv_opt_stop2, destination_loc, stp_depart_time, recharge_time, visited_missions, num_uavs, added_mission_pts, Age_period)
                    if age_period_cnt == 0:
                        Age_period = {nodes: 0 for nodes in mission_points}
                        age_period_cnt += 1
                elif ugv_direction == 2:
                    veh_max_time = uav_strt_time + ugv_distance_calc_auto.distance_btw_ugvstops(fleeting_location[0], destination_loc[0]) // ugv_velocity
                    fitness_val, mission_time, uav_time, total_distance, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp3.main(fleeting_location, ugv_opt_stop1, destination_loc, stp_depart_time, recharge_time, visited_missions, num_uavs, added_mission_pts, Age_period)
                    if age_period_cnt == 0:
                        Age_period = {nodes: 0 for nodes in mission_points}
                        age_period_cnt += 1
                time_on_UGV_charging = 926
                if fitness_val >= 1_000_000:
                    subprob_feasibility_list.append(0)
                else:
                    subprob_feasibility_list.append(1)
                if fitness_val == 0 or len(route_dict) == 2:
                    stp_depart_time = tw_dict[12][0] + time_on_UGV_charging
                    uav_time = tw_dict[12][0] + time_on_UGV_charging
                else:
                    stp_depart_time = uav_time
                if route_dict[-2][0] == ugv_opt_stop1_tuple or route_dict[-2][0] == ugv_opt_stop2_tuple or route_dict[-2][0] == destination_loc or fitness_val == 0:
                    recharge_time = 0 + 1800
                else:
                    recharge_time = time_on_UGV_charging + 1800
                overall_route_list.append(route_dict)
                route_locs_only = [i[0] for i in route_dict]
                for l in route_dict:
                    # if l[0] == fleeting_location[0] or l[0] == destination_loc[0] or l[0] == ugv_opt_stop1[0]:
                    #     continue
                    visited_missions.append(l[0])
                    # visited_missions_so_far.append(l[0])
                ugv_temp_locs = ugv_mission_points((fleeting_location[0], destination_loc[0]), ugv_direction, ordered_pts_from_depot, ordered_pts_rest)
                for i in ugv_temp_locs:
                    visited_missions_so_far.append(i)
                for i in route_locs_only:
                    visited_missions_so_far.append(i)
                for i in ugv_visits_dr_rhg:
                    visited_missions_so_far.append(i)
                for q in Age_period.keys():
                    if q in route_locs_only:
                        # Calculate age for points in the current route
                        idx_num = route_locs_only.index(q)
                        time_reference = uav_strt_time if q == ugv_strt_pt else route_dict[idx_num][1]
                        if q in ugv_visits and ugv_direction == 0:
                            time_reference = max(ugv_visits_time_track[q], route_dict[idx_num][1]) if ordered_pts_from_depot.index(q) < ordered_pts_from_depot.index(destination_loc[0]) else route_dict[idx_num][1]
                        Age_period[q] = uav_time - time_reference
                    else:
                        # Calculate age for points not in the current route
                        Age_period[q] = uav_time - (uav_strt_time if q in visited_missions_so_far else 0)
                print(f"Age_period: {Age_period}")
                if uav_time > veh_max_time or (veh_max_time - uav_time < 60) or (uav_time - veh_max_time < 60):
                    uav_strt_time = uav_time + recharge_time
                else:
                    uav_strt_time = veh_max_time + recharge_time
                total_ugv_uav_visits = ugv_visits+visited_missions_so_far
                total_ugv_uav_visits = [x for i, x in enumerate(total_ugv_uav_visits) if x not in total_ugv_uav_visits[:i]]
                total_ugv_uav_visits = {i: 0 for i in total_ugv_uav_visits}
                for ley, val in ugv_visits_time_track.items():
                    total_ugv_uav_visits[ley] = val
                for i in overall_route_list:
                    for j in i:
                        total_ugv_uav_visits[j[0]] = j[1]
                if destination_loc[0] == ugv_opt_stop1[0] or destination_loc[0] == ugv_opt_stop2[0]:
                    itr_cnt += 1
                    covered_points1 = is_inside_radius(ugv_opt_stop1[0], mission_points, 24750/5280)
                    covered_points2 = is_inside_radius(ugv_opt_stop2[0], mission_points, 24750/5280)
                    intersection = [element for element in covered_points1 if element in covered_points2]
                    covered_points = []
                    if destination_loc[0] == ugv_opt_stop1[0]:
                        covered_points = [element for element in covered_points1 if element not in intersection]
                    elif destination_loc[0] == ugv_opt_stop2[0]:
                        covered_points = [element for element in covered_points2 if element not in intersection]
                    missing_elements = [element for element in covered_points if not any(is_close(element, mission) for mission in visited_missions)]
                    missing_elements = [element for element in missing_elements if element not in ugv_visits_total_missions]
                    print(f"Missing elements: {missing_elements} and ugv_visits: {ugv_visits_total_missions}")
                    if (len(missing_elements) > 0 or fitness_val == 0) and itr_cnt < 3:
                        fleeting_location = destination_loc
                        continue
                    if ugv_direction == 0:
                        ugv_direction = 1
                        stp_travel_time = ugv_distance_calc_auto.distance_from_depotB(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
                        stp_reach_time = stp_travel_time
                        if fitness_val == 0:
                            ugv_wait_time = 0
                        else:
                            ugv_wait_time = route_dict[-1][1] - tw_dict[12][0]
                        ugv_power += ugv_power_consumption_subproblem1.ugv_power(ugv_wait_time, ugv_velocity / 3.281, stp_reach_time)
                        rend_cnt += 1
                    elif ugv_direction == 1:
                        ugv_direction = 2
                        stp_travel_time = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
                        stp_reach_time = stp_travel_time
                        if fitness_val == 0:
                            ugv_wait_time = 0
                        else:
                            ugv_wait_time = route_dict[-1][1] - tw_dict[12][0]
                        ugv_power += ugv_power_consumption_subproblem1.ugv_power(ugv_wait_time, ugv_velocity / 3.281, stp_reach_time)
                        rend_cnt += 1
                    elif ugv_direction == 2:
                        ugv_direction = 1
                        stp_travel_time = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
                        stp_reach_time = stp_travel_time
                        if fitness_val == 0:
                            ugv_wait_time = 0
                        else:
                            ugv_wait_time = route_dict[-1][1] - tw_dict[12][0]
                        ugv_power += ugv_power_consumption_subproblem1.ugv_power(ugv_wait_time, ugv_velocity / 3.281, stp_reach_time)
                        rend_cnt += 1
                    flag = 0
                fleeting_location, ugv_visits_dr_rhg = perch_and_move_agents_age_period(destination_loc, ugv_direction, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
                if ugv_direction == 0:
                    cnt = 0
                    for i in range(cnt, len(ordered_pts_from_depot)):
                        if ordered_pts_from_depot[i] != fleeting_location[0]:
                            cnt += 1
                        else:
                            break
                    uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                    if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_from_depot):
                        destination_loc = ugv_opt_stop1
                    else:
                        destination_loc = [ordered_pts_from_depot[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                    if ordered_pts_from_depot.index(destination_loc[0]) > ordered_pts_from_depot.index(ugv_opt_stop1[0]):
                        destination_loc = ugv_opt_stop1
                elif ugv_direction == 1:
                    cnt = 0
                    for i in range(cnt, len(ordered_pts_rest)):
                        if ordered_pts_rest[i] != fleeting_location[0]:
                            cnt += 1
                        else:
                            break
                    uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                    if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(ordered_pts_rest):
                        destination_loc = ugv_opt_stop2
                    else:
                        destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                    if ordered_pts_rest.index(destination_loc[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                        destination_loc = ugv_opt_stop2
                    stop1 = True
                    stop2 = False
                elif ugv_direction == 2:
                    cnt = 0
                    for i in range(cnt, len(reversed_ordered_pts_rest)):
                        if reversed_ordered_pts_rest[i] != fleeting_location[0]:
                            cnt += 1
                        else:
                            break
                    uav_rendezvous_arrvl_location_sp[rend_cnt] += cnt
                    if uav_rendezvous_arrvl_location_sp[rend_cnt] >= len(reversed_ordered_pts_rest):
                        destination_loc = ugv_opt_stop1
                    else:
                        destination_loc = [reversed_ordered_pts_rest[uav_rendezvous_arrvl_location_sp[rend_cnt]]]
                    if reversed_ordered_pts_rest.index(destination_loc[0]) > reversed_ordered_pts_rest.index(ugv_opt_stop1[0]):
                        destination_loc = ugv_opt_stop1
                    stop1 = False
                    stop2 = True
                for m in route_dict:
                    route_location_visits.append(m)
                visited_missions = []
            for node, age_period in Age_period.items():
                Age_period_for_obj[node].append(age_period)
            subprob_cnt += 1
            route_location_visits.sort(key=lambda x: x[1])
            route_overall_dict.append(route_location_visits)
            # if fitness_val > 1_000_000 and len(visited_missions) != tot_uav_visits_sp1:
            #     penalty += 500_000_000
            # each_stop_ugv_travel_T = stp1_reach_time // count_d_to_stp1
            ugv_ordered_visits = []

            visited_missions_so_far.extend(ugv_visits)
            if len(ugv_visits) > 0:
                ugv_visits.pop()

            # stp1_depart_time = tw_dict[7][1]
            stop1 = True

            route_locations = []
            for k, node in enumerate(route_location_visits):
                route_locations.append(route_location_visits[k][0])

            for n, node in enumerate(route_location_visits):
                if node == ((0, 0), 0):
                    continue
                track_node_age[node[0]].append(route_location_visits[n][1])
            for m, node in enumerate(ugv_visits):
                track_node_age[node].append(ugv_visits_time_track[node])
        for node, age in track_node_age.items():
            track_node_age[node] = list(sorted(set(age)))
        if fitness_val == 0:
            uav_time = stp_depart_time
            penalty += 10000
        for q, nde in Age_period.items():
            if stop1 is True:
                Age_period[q] = Age_period[q] + stp1_return_to_depot_time
            else:
                Age_period[q] = Age_period[q] + stp2_return_to_depot_time
        print("*****************************************************************")
        print(f'The UGV power consumption throughout the mission is: {ugv_power}')
        print("*****************************************************************")
        if stop1 is True:
            stp_depart_time += stp1_return_to_depot_time
        else:
            stp_depart_time += stp2_return_to_depot_time
        fleeting_location = mission_strt_pt
        ugv_direction = 0
        stp_depart_time += 300
        if subprob_cnt <= 1:
            mission_time = stp_depart_time +60000
        else:
            mission_time = stp_depart_time
        ugv_power = 0
        recharge_time = 0
        subprob_cnt_list.append(subprob_cnt)
        subprob_cnt = 0
    for node, age in track_node_age.items():
        track_node_age[node].append(uav_time)
    mission_end_time = uav_time  # this is for the case with UGV flexibility of traveling directly to end point instead of route
    print(f"The mission of Persistent surveillance (Mission 1) ends at: {mission_end_time} seconds")
    # mission_end_time = timet + each_stop_ugv_travel_T
    if ugv_power > ugv_max_capacity:
        penalty += 500_000_000
    rem_fuel = ugv_max_capacity - ugv_power
    # if subprob_cnt < 3:
    #     penalty += 10_000_000

    """Calculation of the penalty cost - aka the age period of the nodes"""
    mission_cost = {}
    for key, val in track_node_age.items():
        cost_val = 0
        if len(val) == 0:
            continue
        elif len(val) == 1:
            for i in range(len(val)):
                # cost_val += scipy.integrate.quad(integral_function, 0, val[i], args=0)[0]
                cost_val += ((mission_end_time//60 - val[i]//60)**2 / 3600)
        else:
            for i in range(len(val)-1):
                # cost_val += scipy.integrate.quad(integral_function, val[i], val[i+1], args=val[i])[0]
                cost_val += ((val[i+1]//60 - val[i]//60)**2 / 3600)
        mission_cost[key] = cost_val

    # Calculate the total cost
    total_cost = 0
    for key1, val1 in mission_cost.items():
        total_cost += val1
    print(f'Age of the nodes: {track_node_age}')
    # print(f"\033[93m Total cost: {total_cost} \033[0m")
    for key, val in track_node_age.items():
        if len(val) < 3:
            if key in ugv_visits:
                penalty += 0
            else:
                penalty += 1_000_000
    max_of_uav_ugv_travel = max(subprob_cnt_list)
    total_cost -= 100 * np.log(max_of_uav_ugv_travel)
    print(f'The total score accquired during this mission is: {total_cost+penalty}')
    param_list = param_list
    feval[tuple(param_list)] = total_cost+penalty
    print(f"Local function evaluated values: {feval}")
    return total_cost+penalty


def persistent_surveillance_optimal(param_list, num_uavs, ugv_stops_dict, dynamic_added_mission_pt_pair, current_loc):
    if dynamic_added_mission_pt_pair[0] not in mission_points and dynamic_added_mission_pt_pair[1] not in mission_points:
        mission_points.extend(dynamic_added_mission_pt_pair)
        print(f"Mission points: {mission_points}")
    route_dict = None
    route_dict2 = None
    route_dict3 = None
    fitness_val = 0
    fitness_val2 = 0
    fitness_val3 = 0
    uav_time = 0
    uav_time2 = 0
    uav_time3 = 0
    nw_wait_time = 0
    se_wait_time = 0
    ugv_travel_time_tot = 0
    ugv_inter_locs = 0

    """Fixed parameters"""
    ugv_max_capacity = 25_010_000  # in J
    ugv_velocity = 15  # in ft/s
    uav_speed = 33  # in ft/s

    """Changing finite states"""
    sp1_flag = 0
    sp2_flag = 0
    sp3_flag = 0

    """Hyperparameter optimization"""

    ugv_travel_time = 0
    stp2_depart_time = 0
    stp1_depart_time = 0
    uav_time2, uav_time3, uav_time4 = 0, 0, 0
    recharge_time = 0
    overall_route_list = []
    strt_pt = [ordered_pts_from_depot[0]]
    # strt_pt = [ordered_pts_from_depot[0]]
    if param_list[0] > len(ugv_stops_dict)+1:
        param_list[0] = len(ugv_stops_dict)+1
    if param_list[0] < 2:
        param_list[0] = 2
    if param_list[1] > 5:  # TODO: Deal with changing these values when initial parameter set values in A-Teams code are changed!!!
        param_list[1] = 5
    if param_list[1] < 2:
        param_list[1] = 2
    if param_list[2] > 7:
        param_list[2] = 7
    if param_list[2] < 2:
        param_list[2] = 2
    if param_list[3] > 7:
        param_list[3] = 7
    if param_list[3] < 2:
        param_list[3] = 2
    # if param_list[5] > 11:
    #     param_list[5] = 11
    # if param_list[5] < 2:
    #     param_list[5] = 2
    ugv_opt_stop1 = [ugv_stops_dict[param_list[0]][0]]
    ugv_opt_stop2 = [ugv_stops_dict[param_list[0]][1]]
    strt_pt_tuple = strt_pt[0]
    depot_to_stp_len = 0
    btw_stps_len = 0
    count_se = 0
    for i in ordered_pts_from_depot:
        if i != ugv_opt_stop1[0]:
            depot_to_stp_len += 1
        else:
            break
    depot_to_stp_len -= 1
    for j in ordered_pts_rest:
        if j != ugv_opt_stop1[0]:
            count_se += 1
            continue
        else:
            for k in range(count_se+1, len(ordered_pts_rest)):
                if ordered_pts_rest[k] != ugv_opt_stop2[0]:
                    btw_stps_len += 1
                else:
                    break
    ugv_opt_stop1_tuple = ugv_opt_stop1[0]
    ugv_opt_stop2_tuple = ugv_opt_stop2[0]
    strt_pt_ft = tuple([i*5280 for i in strt_pt_tuple])
    ugv_opt_stop1_ft = tuple([i*5280 for i in ugv_opt_stop1_tuple])
    ugv_opt_stop2_ft = tuple([i*5280 for i in ugv_opt_stop2_tuple])
    uav_rendezvous_arrvl_location_sp1 = param_list[1]  # in minutes
    uav_rendezvous_arrvl_location_sp2 = param_list[2]
    uav_rendezvous_arrvl_location_sp3 = param_list[3]
    ugv_stop_num = param_list[0]
    ugv_power = 0
    penalty = 0
    # stp1_travel_time = subproblem1_multiUGV_stops_approach.euclidean_distance(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    stp1_travel_time = ugv_distance_calc_auto.distance_from_depotB(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    travel_time_btw_stops = ugv_distance_calc_auto.distance(ugv_opt_stop1_ft, ugv_opt_stop2_ft) // ugv_velocity
    ugv_power_from_nearend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp1_travel_time)
    ugv_power_btw_stops = ((464.8*(ugv_velocity / 3.281) + 356.3)*travel_time_btw_stops)
    stp2_return_to_depot_time = sp1.euclidean_distance(strt_pt_ft, ugv_opt_stop2_ft) // ugv_velocity
    # stp2_return_to_depot_time = (subproblem1_from_strt_ARL.ugv_distance_calc((3.79*5280, 6.71*5280), ugv_opt_stop2_ft) // ugv_velocity) + (subproblem1_from_strt_ARL.ugv_distance_calc((3.79*5280, 6.71*5280), strt_pt_ft) // ugv_velocity)
    # ugv_power_from_farend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp1_travel_time) + ((464.8*(ugv_velocity / 3.281) + 356.3)*travel_time_btw_stops)
    ugv_power_from_farend_to_depot = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp2_return_to_depot_time)
    ugv_power_direct_from_stop2 = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp2_return_to_depot_time)
    stp1_return_to_depot_time = sp1.euclidean_distance(strt_pt_ft, ugv_opt_stop1_ft) // ugv_velocity
    ugv_power_direct_from_stop1 = ((464.8*(ugv_velocity / 3.281) + 356.3)*stp1_return_to_depot_time)
    tot_uav_visits_sp1 = count_se - 1  # to get rid of the first point in ordered_list_rest
    reverse_pts = list(reversed(ordered_pts_rest))
    count_uav_points = 0
    for j in reverse_pts:
        if j != ugv_opt_stop2[0]:
            count_uav_points += 1
            continue
        else:
            break
    tot_uav_visits_sp2 = count_uav_points
    # branch_mid_pt = [(3.79, 6.71)]
    # branch_mid_pt = [(3.72, 6.86)]
    branch_mid_pt = [(3.4, 7.15)]
    additional_pts_for_sp3_cnt = 0
    for i in ordered_pts_from_depot:
        if i != branch_mid_pt[0]:
            additional_pts_for_sp3_cnt += 1
    tot_uav_visits_sp3 = (count_se - 1) + (additional_pts_for_sp3_cnt - 1)   # to not consider Depot B point
    stop1 = False
    stop2 = False
    track_node_age = {nodes: [0] for nodes in mission_points}
    route_overall_dict = []
    subprob_cnt = 0
    # Initially go from start to first UGV stop
    sp1_flag = 1
    fleeting_location = strt_pt
    visited_missions = []
    destination_loc = [ordered_pts_from_depot[uav_rendezvous_arrvl_location_sp1]]
    if ordered_pts_from_depot.index(destination_loc[0]) > ordered_pts_from_depot.index(ugv_opt_stop1[0]):
        destination_loc = ugv_opt_stop1
    route_location_visits = []
    route_location_visits2 = []
    route_location_visits3 = []
    '''Use the following block within the quotes only when doing replanning, else comment it out and make flag = 1 in the previous line instead of 0'''
    flag = 0
    tw_dict = {0: (0, 60), 1: (0, 12284), 2: (0, 12284), 3: (0, 12284), 4: (0, 12284), 5: (0, 12284), 6: (0, 12284), 7: (1284, 11284), 8: (1284, 11284), 9: (1284, 11284), 10: (1284, 11284), 11: (1284, 11284), 12: (1284, 11284), 13: (0, 12284), 14: (0, 12284), 15: (0, 12284), 16: (0, 12284), 17: (0, 12284), 18: (1284, 11284), 19: (0, 11284), 20: (0, 11284), 21: (0, 11284)}
    route_dict = [((0.85, 8.09), 0), ((2.55, 7.82), 275), ((3.44, 8.63), 748), ((3.4, 7.15), 1284), ((4.96, 10.87), 2377), ((3.4, 7.15), 3022), ((3.4, 7.15), 3843)]
    uav_time = 5643
    fitness_val = 1933
    destination_loc = ugv_opt_stop1
    fleeting_location = perch_and_move_agents(destination_loc, sp1_flag, sp2_flag, sp3_flag, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
    recharge_time = 0
    '''Replanning block information ends'''
    subprob_feasibility_list = []
    print(f"The UGV end point locations are: Stop 1 -> {ugv_opt_stop1} and Stop 2 -> {ugv_opt_stop2}")
    while flag:
        fitness_val, mission_time, total_distance, uav_time, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp1.main(fleeting_location[0], ugv_opt_stop1, destination_loc, ugv_stops_dict, visited_missions, num_uavs, dynamic_added_mission_pt_pair)
        if fitness_val >= 1_000_000:
            subprob_feasibility_list.append(0)
        else:
            subprob_feasibility_list.append(1)
        overall_route_list.append(route_dict)
        for l in route_dict:
            if l[0] == fleeting_location[0] or l[0] == destination_loc[0] or l[0] == ugv_opt_stop1[0]:
                continue
            visited_missions.append(l[0])
        if destination_loc[0] == ugv_opt_stop1[0]:
            flag = 0
        if len(visited_missions) == len(ordered_pts_from_depot) - depot_to_stp_len:
            break
        fleeting_location = perch_and_move_agents(destination_loc, sp1_flag, sp2_flag, sp3_flag, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
        destination_loc = ugv_opt_stop1
        for m in route_dict:
            route_location_visits.append(m)
        if route_dict[-2][0] == ugv_opt_stop1_tuple or fitness_val == 0:
            recharge_time = 0 + 1800
        else:
            recharge_time = 926 + 1800
    subprob_cnt += 1
    route_location_visits.sort(key=lambda x: x[1])
    route_overall_dict.append(route_location_visits)
    if fitness_val == 0:
        penalty += 500_000_000_000
    if fitness_val > 1_000_000 and len(visited_missions) != tot_uav_visits_sp1:
        penalty += 500_000_000
    stp1_reach_time = stp1_travel_time
    if fitness_val == 0:
        nw_wait_time = 0
    else:
        nw_wait_time = route_dict[-1][1] - tw_dict[18][0]
    ugv_power += ugv_power_consumption_subproblem1.ugv_power(nw_wait_time, ugv_velocity / 3.281, stp1_reach_time)
    count_d_to_stp1 = 0
    ugv_visits1 = []
    ugv_visits2 = []
    for i, node in enumerate(ordered_pts_from_depot):
        if node == ugv_opt_stop1_tuple:
            break
        else:
            ugv_visits1.append(ordered_pts_from_depot[i+1])
            count_d_to_stp1 += 1
    # each_stop_ugv_travel_T = stp1_reach_time // count_d_to_stp1
    ugv_ordered_visits = []
    ugv_visits_time_track = {}
    timet = 0
    for i in range(len(ugv_visits1)-1):
        each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ugv_visits1[i], ugv_visits1[i+1]) // ugv_velocity
        timet += each_stop_ugv_travel_T
        ugv_visits_time_track[ugv_visits1[i]] = timet

    ugv_visits1.pop()

    # TODO: Change this value and similar other sub-problem values to 'uav_time' based on solution quality obtained. If worse, change, else leave as it is.
    stp1_depart_time = uav_time
    # stp1_depart_time = tw_dict[7][1]
    stop1 = True

    route_locations = []
    for k, node in enumerate(route_location_visits):
        route_locations.append(route_location_visits[k][0])

    for n, node in enumerate(route_location_visits):
        if node == ((0, 0), 0):
            continue
        track_node_age[node[0]].append(route_location_visits[n][1])

    for m, node in enumerate(ugv_visits1):
        track_node_age[node].append(ugv_visits_time_track[node])

    sp1_flag = 0
    visited_missions = []

    # Loop through until UGV power is consumed beyond a certain threshold
    while ugv_power <= ugv_max_capacity:
        ugv_power_check1 = (ugv_max_capacity - ugv_power_from_nearend_to_depot)
        ugv_power_check2 = (ugv_max_capacity - (ugv_power_btw_stops+(ugv_power_from_farend_to_depot)))
        ugv_power_check3 = ((ugv_power_from_nearend_to_depot + ugv_power_btw_stops))
        count_stp1_to_stp2 = 0
        if ugv_power <= ugv_power_check1 and ugv_power <= ugv_power_check2 and stop1 is True and ugv_power_check2 > 0:  # The condition says, if the ugv power consumption is less than the power that is remaining after it has consumed by traveling to respective locations, then continue in the condition.
            subprob_cnt += 1
            sp2_flag = 1
            fleeting_location = perch_and_move_agents(ugv_opt_stop1, sp1_flag, sp2_flag, sp3_flag, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
            end_pt_1_id = 0
            for i in range(len(ordered_pts_rest)):
                if ordered_pts_rest[i] != fleeting_location[0]:
                    end_pt_1_id += 1
                else:
                    break
            uav_rendezvous_arrvl_location_sp2 += end_pt_1_id
            if uav_rendezvous_arrvl_location_sp2 >= ordered_pts_rest.index(ugv_opt_stop2[0]):
                uav_rendezvous_arrvl_location_sp2 = ordered_pts_rest.index(ugv_opt_stop2[0])
            destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp2]]
            '''Use the following block within the quotes only when doing replanning, else comment it out and make flag = 1 in the previous line instead of 0'''
            if current_loc == [(3.4, 7.15)]:
                flag2 = 1
            elif current_loc == [(7.65, 3.44)]:
                flag2 = 0
                file_path = 'Replanning_2.yaml'
                with open(file_path, 'r') as file:
                    yaml_data = yaml.load(file, Loader=yaml.FullLoader)
                tw_dict2 = yaml_data['tw_dict2']
                tw_dict2 = ast.literal_eval(tw_dict2)
                route_dict2 = yaml_data['route_dict2']
                route_dict2 = ast.literal_eval(route_dict2)
                uav_time = yaml_data['uav_time']
                uav_time = ast.literal_eval(uav_time)
                fitness_val2 = yaml_data['fitness_val']
                fitness_val2 = ast.literal_eval(fitness_val2)
                recharge_time = yaml_data['recharge_time']
                recharge_time = ast.literal_eval(recharge_time)
                destination_loc = ugv_opt_stop2
            else:
                flag2 = 1
            '''Replanning ends'''
            timet = stp1_depart_time
            times = []
            times.append(stp1_depart_time//60)
            while flag2:
                ugv_travel_time_tot += 926
                if ordered_pts_rest.index(fleeting_location[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                    fleeting_location = ugv_opt_stop2
                fitness_val2, mission_time2, total_distance2, uav_time, velocity1_2, velocity2_2, route_dict2, route2, tw_dict2, disttravel_track2, ugv_travel_time, ugv_inter_locs = sp2.main(fleeting_location, ugv_opt_stop2, destination_loc, stp1_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs, dynamic_added_mission_pt_pair)
                overall_route_list.append(route_dict2)
                for l in route_dict2:
                    if l[0] == fleeting_location[0] or l[0] == destination_loc[0] or l[0] == ugv_opt_stop2[0]:
                        continue
                    visited_missions.append(l[0])
                if destination_loc[0] == ugv_opt_stop2[0]:
                    flag2 = 0
                if fitness_val2 == 0:
                    stp1_depart_time = tw_dict2[12][0] + 926
                    # times.append(tw_dict2[12][0] // 60)
                    # times = tuple(times)
                    df = pd.DataFrame([(fleeting_location[0]), (destination_loc[0])])
                    # df.insert(0, 'time', times)
                    # df.to_excel('Subproblem2_no_solution_UGV-UAV_rendezvous.xlsx', index=False, engine='openpyxl')
                else:
                    stp1_depart_time = uav_time
                fleeting_location = perch_and_move_agents(destination_loc, sp1_flag, sp2_flag, sp3_flag, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
                destination_loc = ugv_opt_stop2
                for m in route_dict2:
                    route_location_visits2.append(m)
                ugv_travel_time_tot += ugv_travel_time
                if route_dict2[-2][0] == ugv_opt_stop2_tuple or fitness_val2 == 0:
                    recharge_time = 0 + 1800
                else:
                    recharge_time = 926 + 1800
            route_location_visits2.sort(key=lambda x: x[1])
            route_overall_dict.append(route_dict2)
            yaml_data = {'tw_dict2': f'{tw_dict2}', 'route_dict2': f'{route_dict2}', 'uav_time': f'{uav_time}', 'fitness_val': f'{fitness_val2}', 'recharge_time': f'{recharge_time}'}
            file_path = 'Replanning_2.yaml'
            with open(file_path, 'w') as file:
                yaml.dump(yaml_data, file)
            if fitness_val2 == 0 or fitness_val2 > 1_000_000:
                # if fitness_val2 > 1_000_000:
                #     penalty += 500_000_000
                if fitness_val2 == 0:
                    penalty += 500_000_000_000
                # if fitness_val2 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp2-1:
                #     penalty += 500_000_000
            stp2_count = 0
            count = 0
            # stp2_reach_time = tw_dict2[7][0]
            # stp2_depart_time = tw_dict2[7][1]
            stp2_depart_time = uav_time
            if fitness_val2 == 0:
                se_wait_time = 0
            else:
                se_wait_time = route_dict2[-1][1] - tw_dict2[12][0]
            ugv_power += ugv_power_consumption_subproblem2.ugv_power(se_wait_time, ugv_velocity / 3.281, travel_time_btw_stops)
            count = 0
            for i, node in enumerate(ordered_pts_rest):
                count += 1
                if node == ugv_opt_stop1_tuple:
                    for j in range(count+1, len(ordered_pts_rest)):
                        count_stp1_to_stp2 += 1
                        if ordered_pts_rest[j] == ugv_opt_stop2_tuple:
                            break
                        else:
                            ugv_visits2.append(ordered_pts_rest[j])
            # each_stop_ugv_travel_T = ugv_travel_time // count_stp1_to_stp2
            ugv_ordered_visits.append(ugv_visits_time_track)
            ugv_visits_time_track = {}
            for i in range(len(ugv_visits2)-1):
                each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_btw_ugvstops(ugv_visits2[i], ugv_visits2[i+1]) // ugv_velocity
                timet += each_stop_ugv_travel_T
                ugv_visits_time_track[ugv_visits2[i]] = timet
            ugv_visits_time_track[ugv_visits2[-1]] = timet + ugv_distance_calc_auto.distance_btw_ugvstops(ugv_visits2[-2], ugv_visits2[-1]) // ugv_velocity
            stop2 = True
            stop1 = False
            for n, node in enumerate(route_location_visits2):
                if node == ((0, 0), 0):
                    continue
                track_node_age[node[0]].append(route_location_visits2[n][1])
            for m, node in enumerate(ugv_visits2):
                track_node_age[node].append(ugv_visits_time_track[node])
            sp2_flag = 0
            visited_missions = []
            ugv_travel_time_tot = 0
        if ugv_max_capacity - ugv_power > ugv_power_check3 and stop2 is True and ugv_power_check3 > 0:
            subprob_cnt += 1
            sp3_flag = 1
            fleeting_location = perch_and_move_agents(ugv_opt_stop2, sp1_flag, sp2_flag, sp3_flag, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
            reverse_ordered_pts_rest = list(reversed(ordered_pts_rest))
            end_pt_2_id = 0
            for i in range(len(reverse_ordered_pts_rest)):
                if reverse_ordered_pts_rest[i] != fleeting_location[0]:
                    end_pt_2_id += 1
                else:
                    break
            uav_rendezvous_arrvl_location_sp3 += end_pt_2_id
            if uav_rendezvous_arrvl_location_sp3 >= reverse_ordered_pts_rest.index(ugv_opt_stop1[0]):
                uav_rendezvous_arrvl_location_sp3 = reverse_ordered_pts_rest.index(ugv_opt_stop1[0])
            destination_loc = [reverse_ordered_pts_rest[uav_rendezvous_arrvl_location_sp3]]
            flag3 = 1
            timet = stp2_depart_time
            while flag3:
                ugv_travel_time_tot += 926
                if reverse_ordered_pts_rest.index(fleeting_location[0]) > reverse_ordered_pts_rest.index(ugv_opt_stop1[0]):
                    fleeting_location = ugv_opt_stop1
                fitness_val3, mission_time3, total_distance3, uav_time, velocity1_3, velocity2_3, route_dict3, route3, tw_dict3, disttravel_track3, ugv_travel_time, ugv_inter_locs = sp3.main(ugv_opt_stop1, fleeting_location, destination_loc, stp2_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs, dynamic_added_mission_pt_pair)
                overall_route_list.append(route_dict3)
                for l in route_dict3:
                    if l[0] == fleeting_location[0] or l[0] == destination_loc[0] or l[0] == ugv_opt_stop1[0]:
                        continue
                    visited_missions.append(l[0])
                if destination_loc[0] == ugv_opt_stop1[0]:
                    flag3 = 0
                fleeting_location = perch_and_move_agents(destination_loc, sp1_flag, sp2_flag, sp3_flag, ugv_opt_stop1, ugv_opt_stop2, ugv_opt_stop1, ugv_velocity)
                destination_loc = ugv_opt_stop1
                for m in route_dict3:
                    route_location_visits3.append(m)
                if fitness_val3 == 0:
                    stp2_depart_time = tw_dict3[12][0] + 926
                    df = pd.DataFrame([(fleeting_location[0]), (destination_loc[0])])
                    # df.to_excel('Subproblem3_no_solution_UGV-UAV_rendezvous.xlsx', index=False, engine='openpyxl')
                else:
                    stp2_depart_time = uav_time
                ugv_travel_time_tot += ugv_travel_time
                if route_dict3[-2][0] == ugv_opt_stop1_tuple or fitness_val3 == 0:
                    recharge_time = 0 + 1800
                else:
                    recharge_time = 926 + 1800
            route_location_visits3.sort(key=lambda x: x[1])
            route_overall_dict.append(route_location_visits3)
            if fitness_val3 == 0 or fitness_val3 > 1_000_000:
                if fitness_val3 == 0:
                    penalty += 500_000_000_000
                # if fitness_val3 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp3:
                #     penalty += 1_000_000
            stp1_count = 0
            count = 0
            veh_routes = {}
            veh_times = {}
            my_list = []
            # stp1_reach_time = tw_dict3[7][0]
            stp1_depart_time = uav_time
            if fitness_val3 == 0:
                nw_wait_time = 0
            else:
                nw_wait_time = route_dict3[-1][1] - tw_dict3[12][0]
            ugv_power += ugv_power_consumption_subproblem1.ugv_power(nw_wait_time, velocity1_3 / 3.281, ugv_travel_time_tot)
            # each_stop_ugv_travel_T = ugv_travel_time // count_stp1_to_stp2
            ugv_ordered_visits.append(ugv_visits_time_track)
            ugv_visits_time_track = {}
            reversed_ugv_visits2 = list(reversed(ugv_visits2))
            for i in range(len(reversed_ugv_visits2)-1):
                each_stop_ugv_travel_T = ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ugv_visits2[i], reversed_ugv_visits2[i+1]) // ugv_velocity
                timet += each_stop_ugv_travel_T
                ugv_visits_time_track[reversed_ugv_visits2[i]] = timet
            ugv_visits_time_track[reversed_ugv_visits2[-1]] = timet + ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ugv_visits2[-2], reversed_ugv_visits2[-1]) // ugv_velocity
            stop1 = True
            stop2 = False
            for n, node in enumerate(route_dict3):
                if node == ((0, 0), 0):
                    continue
                track_node_age[node[0]].append(route_dict3[n][1])
            for m, node in enumerate(ugv_visits2):
                track_node_age[node].append(ugv_visits_time_track[node])
            if uav_time == 0:
                uav_time = ugv_visits_time_track[reversed_ugv_visits2[-1]]
            sp3_flag = 0
            visited_missions = []
            ugv_travel_time_tot = 0
        else:
            if stop1:
                ugv_power += ugv_power_direct_from_stop1
                # if subprob_cnt == 1 and len(route_dict) > 2:
                #     if route_dict[-1][2] > 0:
                #         uav_time = uav_time - route_dict[-1][2]
                # if subprob_cnt == 2 and len(route_dict2) > 2:
                #     if route_dict2[-1][2] > 0:
                #         uav_time = uav_time - route_dict2[-1][2]
                # if subprob_cnt == 3 and len(route_dict3) > 2:
                #     if route_dict3[-1][2] > 0:
                #         uav_time = uav_time - route_dict3[-1][2]
                uav_time = uav_time + sp3.euclidean_distance(ugv_opt_stop1_ft, strt_pt_ft) // uav_speed
                ugv_visits_time_track = {}
            elif stop2:
                # if ugv_max_capacity - ugv_power >= (ugv_power_direct_from_stop2) and ugv_power + ugv_power_check2 >= ugv_max_capacity:
                ugv_power += ugv_power_direct_from_stop2
                # if subprob_cnt == 1 and len(route_dict) > 2:
                #     if route_dict[-1][2] > 0:
                #         uav_time = uav_time - route_dict[-1][2]
                # if subprob_cnt == 2 and len(route_dict2) > 2:
                #     if route_dict2[-1][2] > 0:
                #         uav_time = uav_time - route_dict2[-1][2]
                # if subprob_cnt == 3 and len(route_dict3) > 2:
                #     if route_dict3[-1][2] > 0:
                #         uav_time = uav_time - route_dict3[-1][2]
                uav_time = uav_time + sp3.euclidean_distance(ugv_opt_stop2_ft, strt_pt_ft) // uav_speed
                ugv_visits_time_track = {}
            break
    track_node_age[strt_pt[0]].append(uav_time)
    print("*****************************************************************")
    print(f'The UGV power consumption throughout the mission is: {ugv_power}')
    print("*****************************************************************")
    mission_end_time = uav_time  # this is for the case with UGV flexibility of traveling directly to end point instead of route
    print(f"The mission of Persistent surveillance (Mission 1) ends at: {mission_end_time} seconds")
    # mission_end_time = timet + each_stop_ugv_travel_T
    if ugv_power > ugv_max_capacity:
        penalty += 500_000_000
    rem_fuel = ugv_max_capacity - ugv_power
    if subprob_cnt < 3:
        penalty += 10_000_000

    """Calculation of the penalty cost - aka the age period of the nodes"""
    mission_cost = {}
    for key, val in track_node_age.items():
        if key == (4.1, 11.51):
            continue
        track_node_age[key].append(mission_end_time)
        cost_val = 0
        if len(val) == 0:
            continue
        elif len(val) == 1:
            for i in range(len(val)):
                # cost_val += scipy.integrate.quad(integral_function, 0, val[i], args=0)[0]
                cost_val += ((mission_end_time//60 - val[i]//60)**3 / 2700)
        else:
            for i in range(len(val)-1):
                # cost_val += scipy.integrate.quad(integral_function, val[i], val[i+1], args=val[i])[0]
                cost_val += ((val[i+1]//60 - val[i]//60)**3 / 2700)
        mission_cost[key] = cost_val

    # Calculate the total cost
    total_cost = 0
    for key1, val1 in mission_cost.items():
        total_cost += val1
    # for key, val in track_node_age.items():
    #     if len(val) < 3:
    #         if key in ugv_visits:
    #             penalty += 0
    #         else:
    #             penalty += 1_000_000
    print(f'Age of the nodes: {track_node_age}')
    print(f'The total score accquired during this mission is: {mission_end_time+penalty}')
    return mission_end_time - uav_time, mission_end_time, ugv_velocity, ugv_velocity, overall_route_list, route_dict, route_dict2, route_dict3, subprob_feasibility_list, ugv_ordered_visits, mission_end_time+penalty


if __name__ == "__main__":
    # LHS - Actual whole scenario with 48 points
    # RHS - Reduced scenario for hardware implementation
    # param_list = [23, 2, 2, 2]  # optimal solution of whole scenario (LHS), but translated that LHS solution to simulate reduced map output (RHS) - RS = 1
    # param_list = [20, 2, 3, 4]  # optimal solution of whole scenario (LHS), but translated that LHS solution to simulate reduced map output (RHS) - RS = 111
    # param_list = [27, 2, 3, 2]  # optimal solution of whole scenario (LHS), but translated that LHS solution to simulate reduced map output (RHS) - RS = 10 (not exactly translated - the rendezvous locations might be off by one location)
    # param_list = [17, 5, 5, 7]  # optimal UGV solution for the reduced scenario - RS = 1
    param_list = [2, 3, 3, 4]
    # param_list = [22, 4, 2, 2]  # optimal UGV solution for the reduced scenario - RS = 10
    # param_list = [45, 5, 9, 6]
    # Some of the optimal parameter lists obtained from various initializations and formulations
    # param_list = [2, 5, 8, 5]
    # param_list = [5, 4, 5, 7]
    # param_list = [41, 6, 9, 10]
    # param_list = [3, 4, 10, 4]
    # param_list = [2, 4, 7, 9]
    # param_list = [5, 4, 5, 7]

    num_uavs = 1
    ugv_stops_dict = {}
    ordered_pts_from_depot = pd.read_csv('Hardware experimental scaled up scenario in miles depotb ordered.csv')
    ordered_pts_rest_df = pd.read_csv('Hardware experimental scaled up scenario in miles ordered.csv')
    ordered_pts_from_depot = ordered_pts_from_depot.values.tolist()
    ordered_pts_rest = ordered_pts_rest_df.values.tolist()
    ordered_pts_from_depot = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot]
    ordered_pts_rest = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_rest]
    primary_rendz_locs = [(3.4, 7.15), (7.65, 3.44)]
    current_loc = [(3.4, 7.15)]
    if current_loc[0] == primary_rendz_locs[0]:
        visited_missions_so_far = [(0.85, 8.09), (2.55, 7.82), (3.44, 8.63), (4.96, 10.87), (3.4, 7.15)]
    elif current_loc[[0] == primary_rendz_locs[1]]:
        visited_missions_so_far = [(5.05, 5.86), (9.17, 2.5), (10.55, 0.45), (11.18, 0.76), (11.0, 1.25), (7.65, 3.44)]
    else:
        visited_missions_so_far = []
    ugv_Stops = minimum_set_cover_UGV_hyperparam_refined_replanning.main(ordered_pts_rest_df, 'Hardware experimental scaled up case study scenario in miles.csv', 100, visited_missions_so_far, current_loc)
    if len(visited_missions_so_far) > 0:
        for i in ugv_Stops:
            i.insert(0, current_loc[0])
    # Updated UGV end pairs for replanning
    # ugv_stops_dict = {}
    # ugv_Stops = [[(3.43, 6.9), (7.45, 3.86)], [(3.43, 6.9), (7.62, 3.69)], [(3.43, 6.9), (7.7, 3.61)], [(3.43, 6.9), (8.04, 3.28)]]
    cnt = 0
    for i in range(len(ugv_Stops)):
        if ugv_Stops[i][0] == (4.11, 6.57):
            continue
        ugv_stops_dict[cnt+2] = ugv_Stops[i]
        cnt += 1
    total_cost = persistent_surveillance_replanning(param_list, num_uavs, ugv_stops_dict, ((3.0, 5.0), (5.25, 8.5)), current_loc)
