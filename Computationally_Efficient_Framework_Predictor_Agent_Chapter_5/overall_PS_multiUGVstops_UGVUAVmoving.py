import pandas as pd
import numpy as np
import minimum_set_cover_UGV_hyperparam_refined
import ugv_power_consumption
import ugv_power_consumption_subproblem1
import ugv_power_consumption_subproblem2
# import subproblem1_multiUGV_stops_approach
# import subproblem2_multiUGV_stops_approach
# import subproblem3_multiUGV_stops_approach
# import subproblem4_multiUGV_stops_approach
import sp1_multiUGVstops_ateams_movingUGV_justend_rch_pts as sp1
# import sp1_multiUGVstops_ateams_movingUGV as sp1
import sp2_multiUGV_stops_ateams_movingUGV_justend_rch_pts as sp2
# import sp2_multiUGV_stops_ateams_movingUGV as sp2
import sp3_multiUGV_stops_ateams_movingUGV_justend_rch_pts as sp3
# import sp3_multiUGV_stops_ateams_movingUGV as sp3
# import sp4_multiUGV_stops_ateams as sp4
import yaml
import random
import csv
import copy
import time
import matplotlib.pyplot as plt
import scipy
import ugv_distance_calc_auto


mission_points = pd.read_excel('ARL corridor limited data points.xlsx', engine='openpyxl')
ordered_pts_from_depot = pd.read_excel('ARL corridor points ordered DepotB.xlsx', engine='openpyxl')
ordered_pts_rest = pd.read_excel('ARL corridor points ordered.xlsx', engine='openpyxl')
# mission_points = pd.read_excel('Mission locations hardware - scaledup for simulation.xlsx', engine='openpyxl')
# ordered_pts_from_depot = pd.read_excel('Mission locations hardware - ordered DepotB.xlsx', engine='openpyxl')
# ordered_pts_rest = pd.read_excel('Mission locations hardware - ordered.xlsx', engine='openpyxl')
mission_points = mission_points.values.tolist()
ordered_pts_from_depot = ordered_pts_from_depot.values.tolist()
ordered_pts_rest = ordered_pts_rest.values.tolist()
mission_points = [(round(m[0], 2), round(m[1], 2)) for m in mission_points]
ordered_pts_from_depot = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot]
ordered_pts_rest = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_rest]


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


def persistent_surveillance(param_list, num_uavs, ugv_stops_dict):
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
    if param_list[0] > len(ugv_stops_dict):
        param_list[0] = len(ugv_stops_dict)
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
    branch_mid_pt = [(3.79, 6.71)]
    # branch_mid_pt = [(3.72, 6.86)]
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
    flag = 1
    subprob_feasibility_list = []
    print(f"The UGV end point locations are: Stop 1 -> {ugv_opt_stop1} and Stop 2 -> {ugv_opt_stop2}")
    while flag:
        fitness_val, mission_time, total_distance, uav_time, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp1.main(fleeting_location[0], ugv_opt_stop1, destination_loc, ugv_stops_dict, visited_missions, num_uavs)
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
            recharge_time = 0
        else:
            recharge_time = 926
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
    ugv_power += ugv_power_consumption_subproblem1.ugv_power(nw_wait_time, velocity1 / 3.281, stp1_reach_time)
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
            if uav_rendezvous_arrvl_location_sp2 >= len(ordered_pts_rest):
                uav_rendezvous_arrvl_location_sp2 = len(ordered_pts_rest)-1
            destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp2]]
            flag2 = 1
            timet = stp1_depart_time
            times = []
            times.append(stp1_depart_time//60)
            while flag2:
                ugv_travel_time_tot += 926
                if ordered_pts_rest.index(fleeting_location[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                    fleeting_location = ugv_opt_stop2
                fitness_val2, mission_time2, total_distance2, uav_time, velocity1_2, velocity2_2, route_dict2, route2, tw_dict2, disttravel_track2, ugv_travel_time, ugv_inter_locs = sp2.main(fleeting_location, ugv_opt_stop2, destination_loc, stp1_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs)
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
                    recharge_time = 0
                else:
                    recharge_time = 926
            route_location_visits2.sort(key=lambda x: x[1])
            route_overall_dict.append(route_dict2)
            if fitness_val2 == 0 or fitness_val2 > 1_000_000:
                # if fitness_val2 > 1_000_000:
                #     penalty += 500_000_000
                if fitness_val2 == 0:
                    penalty += 500_000_000_000
                if fitness_val2 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp2-1:
                    penalty += 500_000_000
            stp2_count = 0
            count = 0
            # stp2_reach_time = tw_dict2[7][0]
            # stp2_depart_time = tw_dict2[7][1]
            stp2_depart_time = uav_time
            if fitness_val2 == 0:
                se_wait_time = 0
            else:
                se_wait_time = route_dict2[-1][1] - tw_dict2[12][0]
            ugv_power += ugv_power_consumption_subproblem2.ugv_power(se_wait_time, velocity2_2 / 3.281, ugv_travel_time_tot)
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
            if uav_rendezvous_arrvl_location_sp3 >= len(ordered_pts_rest):
                uav_rendezvous_arrvl_location_sp3 = len(ordered_pts_rest)-1
            destination_loc = [reverse_ordered_pts_rest[uav_rendezvous_arrvl_location_sp3]]
            flag3 = 1
            timet = stp2_depart_time
            while flag3:
                ugv_travel_time_tot += 926
                if reverse_ordered_pts_rest.index(fleeting_location[0]) > reverse_ordered_pts_rest.index(ugv_opt_stop1[0]):
                    fleeting_location = ugv_opt_stop1
                fitness_val3, mission_time3, total_distance3, uav_time, velocity1_3, velocity2_3, route_dict3, route3, tw_dict3, disttravel_track3, ugv_travel_time, ugv_inter_locs = sp3.main(ugv_opt_stop1, fleeting_location, destination_loc, stp2_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs)
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
                    recharge_time = 0
                else:
                    recharge_time = 926
            route_location_visits3.sort(key=lambda x: x[1])
            route_overall_dict.append(route_location_visits3)
            if fitness_val3 == 0 or fitness_val3 > 1_000_000:
                if fitness_val3 == 0:
                    penalty += 500_000_000_000
                if fitness_val3 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp3:
                    penalty += 1_000_000
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
                uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop1_ft, strt_pt_ft) // uav_speed
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
                uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop2_ft, strt_pt_ft) // uav_speed
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
    print(f'Age of the nodes: {track_node_age}')
    print(f'The total score accquired during this mission is: {mission_end_time+penalty}')
    return mission_end_time - uav_time, mission_end_time, ugv_velocity, ugv_velocity, overall_route_list, route_dict, route_dict2, route_dict3, subprob_feasibility_list, ugv_ordered_visits, mission_end_time+penalty


def persistent_surveillance_optimal(param_list, num_uavs, ugv_stops_dict):
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
    if param_list[0] > len(ugv_stops_dict):
        param_list[0] = len(ugv_stops_dict)
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
    branch_mid_pt = [(3.79, 6.71)]
    # branch_mid_pt = [(3.72, 6.86)]
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
    flag = 1
    subprob_feasibility_list = []
    print(f"The UGV end point locations are: Stop 1 -> {ugv_opt_stop1} and Stop 2 -> {ugv_opt_stop2}")
    while flag:
        fitness_val, mission_time, total_distance, uav_time, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp1.main(fleeting_location[0], ugv_opt_stop1, destination_loc, ugv_stops_dict, visited_missions, num_uavs)
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
            recharge_time = 0
        else:
            recharge_time = 926
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
    ugv_power += ugv_power_consumption_subproblem1.ugv_power(nw_wait_time, velocity1 / 3.281, stp1_reach_time)
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
            if uav_rendezvous_arrvl_location_sp2 >= len(ordered_pts_rest):
                uav_rendezvous_arrvl_location_sp2 = len(ordered_pts_rest)-1
            destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp2]]
            flag2 = 1
            timet = stp1_depart_time
            times = []
            times.append(stp1_depart_time//60)
            while flag2:
                ugv_travel_time_tot += 926
                if ordered_pts_rest.index(fleeting_location[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                    fleeting_location = ugv_opt_stop2
                fitness_val2, mission_time2, total_distance2, uav_time, velocity1_2, velocity2_2, route_dict2, route2, tw_dict2, disttravel_track2, ugv_travel_time, ugv_inter_locs = sp2.main(fleeting_location, ugv_opt_stop2, destination_loc, stp1_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs)
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
                    recharge_time = 0
                else:
                    recharge_time = 926
            route_location_visits2.sort(key=lambda x: x[1])
            route_overall_dict.append(route_dict2)
            if fitness_val2 == 0 or fitness_val2 > 1_000_000:
                # if fitness_val2 > 1_000_000:
                #     penalty += 500_000_000
                if fitness_val2 == 0:
                    penalty += 500_000_000_000
                if fitness_val2 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp2-1:
                    penalty += 500_000_000
            stp2_count = 0
            count = 0
            # stp2_reach_time = tw_dict2[7][0]
            # stp2_depart_time = tw_dict2[7][1]
            stp2_depart_time = uav_time
            if fitness_val2 == 0:
                se_wait_time = 0
            else:
                se_wait_time = route_dict2[-1][1] - tw_dict2[12][0]
            ugv_power += ugv_power_consumption_subproblem2.ugv_power(se_wait_time, velocity2_2 / 3.281, ugv_travel_time_tot)
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
            if uav_rendezvous_arrvl_location_sp3 >= len(ordered_pts_rest):
                uav_rendezvous_arrvl_location_sp3 = len(ordered_pts_rest)-1
            destination_loc = [reverse_ordered_pts_rest[uav_rendezvous_arrvl_location_sp3]]
            flag3 = 1
            timet = stp2_depart_time
            while flag3:
                ugv_travel_time_tot += 926
                if reverse_ordered_pts_rest.index(fleeting_location[0]) > reverse_ordered_pts_rest.index(ugv_opt_stop1[0]):
                    fleeting_location = ugv_opt_stop1
                fitness_val3, mission_time3, total_distance3, uav_time, velocity1_3, velocity2_3, route_dict3, route3, tw_dict3, disttravel_track3, ugv_travel_time, ugv_inter_locs = sp3.main(ugv_opt_stop1, fleeting_location, destination_loc, stp2_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs)
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
                    recharge_time = 0
                else:
                    recharge_time = 926
            route_location_visits3.sort(key=lambda x: x[1])
            route_overall_dict.append(route_location_visits3)
            if fitness_val3 == 0 or fitness_val3 > 1_000_000:
                if fitness_val3 == 0:
                    penalty += 500_000_000_000
                if fitness_val3 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp3:
                    penalty += 1_000_000
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
                uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop1_ft, strt_pt_ft) // uav_speed
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
                uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop2_ft, strt_pt_ft) // uav_speed
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
    print(f'Age of the nodes: {track_node_age}')
    print(f'The total score accquired during this mission is: {mission_end_time+penalty}')
    return mission_end_time - uav_time, mission_end_time, ugv_velocity, ugv_velocity, overall_route_list, route_dict, route_dict2, route_dict3, subprob_feasibility_list, ugv_ordered_visits, mission_end_time+penalty


def persistent_surveillance_for_local_improver_agent(param_list, num_uavs, ugv_stops_dict, feval={}):
    print("++++++++++++++++++++++++++++++ Local optimization has started +++++++++++++++++++++++++++++++")
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
    if param_list[0] > len(ugv_stops_dict):
        param_list[0] = len(ugv_stops_dict)
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
    ugv_opt_stop1 = [ugv_stops_dict[int(param_list[0])][0]]
    ugv_opt_stop2 = [ugv_stops_dict[int(param_list[0])][1]]
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
    uav_rendezvous_arrvl_location_sp1 = int(param_list[1])
    uav_rendezvous_arrvl_location_sp2 = int(param_list[2])
    uav_rendezvous_arrvl_location_sp3 = int(param_list[3])
    ugv_stop_num = int(param_list[0])
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
    branch_mid_pt = [(3.79, 6.71)]
    # branch_mid_pt = [(3.72, 6.86)]
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
    flag = 1
    print(f"The UGV end point locations are: Stop 1 -> {ugv_opt_stop1} and Stop 2 -> {ugv_opt_stop2}")
    while flag:
        fitness_val, mission_time, total_distance, uav_time, velocity1, velocity2, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs = sp1.main(fleeting_location[0], ugv_opt_stop1, destination_loc, ugv_stops_dict, visited_missions, num_uavs)
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
            recharge_time = 0
        else:
            recharge_time = 926
    subprob_cnt += 1
    route_location_visits.sort(key=lambda x: x[1])
    route_overall_dict.append(route_location_visits)
    if fitness_val == 0:
        penalty += 500_000_000_000
    if fitness_val > 1_000_000 and len(visited_missions) != tot_uav_visits_sp1:
        penalty += 500_000_000
        # for key, value in veh_times.items():
        #     if maximum in value:
        #         if veh_routes[key][-2][0] == ugv_opt_stop1_tuple:
        #             recharge_time = 0  # in seconds
        #         else:
        #             recharge_time = 926  # in seconds
        #     else:
        #         continue
    stp1_reach_time = stp1_travel_time
    if fitness_val == 0:
        nw_wait_time = 0
    else:
        nw_wait_time = route_dict[-1][1] - tw_dict[18][0]
    ugv_power += ugv_power_consumption_subproblem1.ugv_power(nw_wait_time, velocity1 / 3.281, stp1_reach_time)
    # for i in ordered_pts_rest:
    #     if i != ugv_opt_stop1_tuple:
    #         continue
    #     else:
    #         if ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ugv_opt_stop1_tuple, i) >= 900:
    #             ugv_opt_stop1 = [i]
    #             break
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
            if uav_rendezvous_arrvl_location_sp2 >= len(ordered_pts_rest):
                uav_rendezvous_arrvl_location_sp2 = len(ordered_pts_rest)-1
            destination_loc = [ordered_pts_rest[uav_rendezvous_arrvl_location_sp2]]
            flag2 = 1
            timet = stp1_depart_time
            times = []
            times.append(stp1_depart_time//60)
            while flag2:
                ugv_travel_time_tot += 926
                if ordered_pts_rest.index(fleeting_location[0]) > ordered_pts_rest.index(ugv_opt_stop2[0]):
                    fleeting_location = ugv_opt_stop2
                fitness_val2, mission_time2, total_distance2, uav_time, velocity1_2, velocity2_2, route_dict2, route2, tw_dict2, disttravel_track2, ugv_travel_time, ugv_inter_locs = sp2.main(fleeting_location, ugv_opt_stop2, destination_loc, stp1_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs)
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
                    recharge_time = 0
                else:
                    recharge_time = 926
            route_location_visits2.sort(key=lambda x: x[1])
            route_overall_dict.append(route_dict2)
            if fitness_val2 == 0 or fitness_val2 > 1_000_000:
                # if fitness_val2 > 1_000_000:
                #     penalty += 500_000_000
                if fitness_val2 == 0:
                    penalty += 500_000_000_000
                if fitness_val2 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp2-1:
                    penalty += 500_000_000
            stp2_count = 0
            count = 0
            # stp2_reach_time = tw_dict2[7][0]
            # stp2_depart_time = tw_dict2[7][1]
            stp2_depart_time = uav_time
            if fitness_val2 == 0:
                se_wait_time = 0
            else:
                se_wait_time = route_dict2[-1][1] - tw_dict2[12][0]
            ugv_power += ugv_power_consumption_subproblem2.ugv_power(se_wait_time, velocity2_2 / 3.281, ugv_travel_time_tot)
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
            if uav_rendezvous_arrvl_location_sp3 >= len(ordered_pts_rest):
                uav_rendezvous_arrvl_location_sp3 = len(ordered_pts_rest)-1
            destination_loc = [reverse_ordered_pts_rest[uav_rendezvous_arrvl_location_sp3]]
            flag3 = 1
            timet = stp2_depart_time
            while flag3:
                ugv_travel_time_tot += 926
                if reverse_ordered_pts_rest.index(fleeting_location[0]) > reverse_ordered_pts_rest.index(ugv_opt_stop1[0]):
                    fleeting_location = ugv_opt_stop1
                fitness_val3, mission_time3, total_distance3, uav_time, velocity1_3, velocity2_3, route_dict3, route3, tw_dict3, disttravel_track3, ugv_travel_time, ugv_inter_locs = sp3.main(ugv_opt_stop1, fleeting_location, destination_loc, stp2_depart_time, recharge_time, ugv_stops_dict, visited_missions, num_uavs)
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
                    recharge_time = 0
                else:
                    recharge_time = 926
            route_location_visits3.sort(key=lambda x: x[1])
            route_overall_dict.append(route_location_visits3)
            if fitness_val3 == 0 or fitness_val3 > 1_000_000:
                if fitness_val3 == 0:
                    penalty += 500_000_000_000
                if fitness_val3 > 1_000_000 and len(visited_missions) != tot_uav_visits_sp3:
                    penalty += 1_000_000
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
                uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop1_ft, strt_pt_ft) // uav_speed
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
                uav_time = uav_time + sp1.euclidean_distance(ugv_opt_stop2_ft, strt_pt_ft) // uav_speed
                ugv_visits_time_track = {}
            break
    track_node_age[strt_pt[0]].append(uav_time)
    print("*****************************************************************")
    print(f'The UGV power consumption throughout the mission is: {ugv_power}')
    print("*****************************************************************")
    mission_end_time = uav_time  # this is for the case with UGV flexibility where UGV can directly travel to end node
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
    print(f'Age of the nodes: {track_node_age}')
    print(f'The total score accquired during this mission is: {mission_end_time+penalty}')
    param_list = param_list
    feval[tuple(param_list)] = mission_end_time+penalty
    print(f"Local function evaluated values: {feval}")
    return mission_end_time+penalty


if __name__ == "__main__":
    # LHS - Actual whole scenario with 48 points
    # RHS - Reduced scenario for hardware implementation
    # param_list = [23, 2, 2, 2]  # optimal solution of whole scenario (LHS), but translated that LHS solution to simulate reduced map output (RHS) - RS = 1
    # param_list = [20, 2, 3, 4]  # optimal solution of whole scenario (LHS), but translated that LHS solution to simulate reduced map output (RHS) - RS = 111
    # param_list = [27, 2, 3, 2]  # optimal solution of whole scenario (LHS), but translated that LHS solution to simulate reduced map output (RHS) - RS = 10 (not exactly translated - the rendezvous locations might be off by one location)
    param_list = [155, 4, 4, 5]
    '''param_list = [18, 3, 2, 5]  # optimal UGV solution for the reduced scenario - RS = 1'''
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
    mission_points_ordered_df = pd.read_excel('ARL corridor points ordered.xlsx', engine='openpyxl')
    mission_points_df = pd.read_excel('ARL corridor limited data points.xlsx', engine='openpyxl')
    # mission_points_ordered_df = pd.read_excel('Mission locations hardware - ordered.xlsx', engine='openpyxl')
    # mission_points_df = pd.read_excel('Mission locations hardware - scaledup for simulation.xlsx', engine='openpyxl')
    mission_points = mission_points_df.values.tolist()
    mission_points = [(round(m[0], 2), round(m[1], 2)) for m in mission_points]
    # for i, pt in enumerate(mission_points):
    #     if pt == (3.02, 8.87):
    #         mission_points[i] = (3.01, 8.87)
    #     elif pt == (3.1, 7.23):
    #         mission_points[i] = (3.1, 7.24)
    #     elif pt == (10.75, 1.78):
    #         mission_points[i] = (10.75, 1.79)
    ugv_Stops = minimum_set_cover_UGV_hyperparam_refined.main(mission_points_ordered_df, 'Case_Study_scenario.csv')
    # Updated UGV end pairs for replanning
    # ugv_stops_dict = {}
    # ugv_Stops = [[(3.43, 6.9), (7.45, 3.86)], [(3.43, 6.9), (7.62, 3.69)], [(3.43, 6.9), (7.7, 3.61)], [(3.43, 6.9), (8.04, 3.28)]]
    for i in range(len(ugv_Stops)):
        ugv_stops_dict[i+2] = ugv_Stops[i]
    total_cost = persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
