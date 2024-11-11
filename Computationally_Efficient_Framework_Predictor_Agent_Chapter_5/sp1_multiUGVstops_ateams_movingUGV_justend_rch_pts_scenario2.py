"""UGV travels at a speed of 3 m/s. UAV travels at a speed of 10 m/s.
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
from time import time
import random
import ugv_distance_calc_auto_scenario2 as ugv_distance_calc_auto
import sys


ordered_pts_from_depot = pd.read_csv('Scenario 6 ordered from depot.csv')
ordered_pts_rest = pd.read_csv('Scenario 6 ordered rest.csv')
ordered_pts_from_depot_second = pd.read_csv('Scenario 6 ordered from depot second.csv')
# ordered_pts_from_depot = pd.read_excel('Mission locations hardware - ordered DepotB.xlsx', engine='openpyxl')
# ordered_pts_rest = pd.read_excel('Mission locations hardware - ordered.xlsx', engine='openpyxl')
ordered_pts_from_depot = ordered_pts_from_depot.values.tolist()
ordered_pts_rest = ordered_pts_rest.values.tolist()
ordered_pts_from_depot_second = ordered_pts_from_depot_second.values.tolist()
ordered_pts_from_depot = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot]
ordered_pts_rest = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_rest]
reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))
ordered_pts_from_depot_second = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot_second]


def get_split_points(lst, m, max_offset=0):
    n = len(lst)
    m = int(m)
    indices = [0] + [i*n//m for i in range(1, m+1)] + [n]
    offsets = [random.uniform(-max_offset, max_offset)*n//m for _ in range(m+1)]
    indices = [int(indices[i] + offsets[i]) for i in range(m+1)]
    return indices[1:]


def perch_and_move_agents(location, ugv_velocity):
    recharge_constant_time = 926
    uav_act_depart_location = []
    cumul_dist = 0
    cnt = 0
    for i in ordered_pts_from_depot:
        if i != location:
            cnt += 1
            continue
        else:
            for j in range(cnt, len(ordered_pts_from_depot)):
                if j == len(ordered_pts_from_depot)-1:
                    continue
                temp = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ordered_pts_from_depot[j], ordered_pts_from_depot[j+1], ordered_pts_from_depot) // ugv_velocity
                cumul_dist += temp
                if cumul_dist >= recharge_constant_time:
                    uav_act_depart_location.append(ordered_pts_from_depot[j])
                    break
            if len(uav_act_depart_location) == 0:
                uav_act_depart_location = [ordered_pts_from_depot[-2]]
        break
    return uav_act_depart_location


def create_data_model(strt_pt, sp1_end_point, stp1_ugv_stop_list, ugv_stops_dict, visited_missions, num_uavs, data_flag, ordered_pts_rest):
    global ordered_pts_from_depot
    n_depot_1 = 6
    n_stop_1 = 6
    n_depot_2 = 6
    ugv_velocity = 15
    data = {}
    fuel_capacity = 287700
    perch_move_agents_dict = {}
    if data_flag == 1:
        ordered_pts_from_depot = ordered_pts_from_depot_second
    for i in ordered_pts_from_depot:
        perch_move_agents_dict[i] = perch_and_move_agents(i, ugv_velocity)
    # df = pd.read_excel('ARL corridor reduced points ordered DepotB.xlsx')
    if data_flag == 1:
        df = pd.read_csv('Scenario 6 ordered from depot second.csv')
    else:
        df = pd.read_csv('Scenario 6 ordered from depot.csv')
    df_list = df.values.tolist()
    df_list = [(round(m[0], 2), round(m[1], 2)) for m in df_list]
    for i, pt in enumerate(df_list):
        if pt == (3.02, 8.87):
            df_list[i] = (3.01, 8.87)
        elif pt == (3.1, 7.23):
            df_list[i] = (3.1, 7.24)
        elif pt == (10.75, 1.78):
            df_list[i] = (10.75, 1.79)
    strt_pt_ft = (strt_pt[0]*5280, strt_pt[1]*5280)
    # n_sum_rch_stops = n_depot_1 + n_stop_1 + n_depot_2

    """Mission points set when UGV goes directly from start node to stop 1"""
    ordered_mission_pts_stp1_stop = []
    if data_flag == 1:
        ordered_pts_rest = reversed_ordered_pts_rest
        for i in range(len(ordered_pts_rest)//2):
            ordered_mission_pts_stp1_stop.append(ordered_pts_rest[i])
    else:
        for i in range(len(ordered_pts_rest)//2):
            ordered_mission_pts_stp1_stop.append(ordered_pts_rest[i])
    # ordered_mission_pts_stp1_stop = [(4.29, 11.21), (4.01, 10.83), (3.74, 10.45),
    #                                  (3.54, 10.04), (3.42, 9.61), (3.29, 9.23), (3.01, 8.87), (2.61, 8.34), (2.43, 7.91), (2.77, 7.57), (3.1, 7.24),
    #                                  (3.43, 6.9), (3.79, 6.71), (4.11, 6.59), (4.55, 6.43), (4.66, 6.4), (4.98, 6.28), (5.37, 6.07),
    #                                  (5.77, 5.87)]
    print(len(ordered_mission_pts_stp1_stop))

    stp1_ugv_stop_list_copy = []
    for i in range(len(stp1_ugv_stop_list)):
        stp1_ugv_stop_list_copy.append(stp1_ugv_stop_list[i])
    stp1_ugv_stop = stp1_ugv_stop_list_copy.pop()
    stop1 = False
    stp1_ugv_stp_ft_list = [stp1_ugv_stop[i]*5280 for i in range(len(stp1_ugv_stop))]
    stp1_ugv_stop_ft = tuple(stp1_ugv_stp_ft_list)
    depot_b_dist = ugv_distance_calc_auto.distance_from_depotB((ordered_pts_from_depot[0][0]*5280, ordered_pts_from_depot[0][1]*5280), stp1_ugv_stop_ft, ordered_pts_from_depot)
    print(strt_pt, stp1_ugv_stop)
    distance_btw_each_stop = 0
    _locations = []
    _locs_depotb = []
    count = 0
    count_se = 0
    for j in range(len(df_list)):
        if df_list[j] != strt_pt and count_se == 0:
            continue
        count_se += 1
        count += 1
        if df_list[j] == stp1_ugv_stop:
            for k in range(df_list.index(strt_pt), df_list.index(strt_pt)+count):
                _locs_depotb.append(df_list[k])
    act_ugv_locs = [strt_pt, ordered_pts_from_depot[0], ordered_pts_from_depot[0]]
    _locations_wo_end_pts = []
    """First version: Where just two UGV points are considered as recharging points"""
    _locations = ([strt_pt] +
                  [ordered_pts_from_depot[0]] * n_depot_1 +
                  stp1_ugv_stop_list * n_stop_1 +
                  [ordered_pts_from_depot[0]] * (n_depot_2 - 1) +
                  stp1_ugv_stop_list
                  )
    n_sum_rch_stops = n_depot_1 + n_stop_1 + n_depot_2
    print(_locs_depotb)
    mission_locations = []
    count = 0
    count_se = 0
    for i in range(len(ordered_mission_pts_stp1_stop)):
        count += 1
        if ordered_mission_pts_stp1_stop[i] == sp1_end_point[0]:
            for j in range(count-1):
                mission_locations.append(ordered_mission_pts_stp1_stop[j])
            break
        else:
            continue
    for o in visited_missions:
        if o in mission_locations:
            mission_locations.remove(o)
    total_mission_pts = len(mission_locations)
    for i in range(len(mission_locations)):
        _locations.append(mission_locations[i])
    data["coordinates"] = _locations
    data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]
    data["num_locations"] = len(data["locations"])
    # depot_b_to_nw_velocity = 10
    # print(depot_b_to_nw_velocity, velocity)
    uav_strt_time = (ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ordered_pts_from_depot[0], strt_pt, ordered_pts_from_depot) // ugv_velocity)
    nw_stop_tw_1 = int(depot_b_dist // ugv_velocity)
    nw_stop_tw_2 = int(nw_stop_tw_1 + 60)
    if stp1_ugv_stop_list[0] == sp1_end_point[0]:
        veh_max_time = int(nw_stop_tw_1 + 10000)
        """First version: Where just two UGV points are considered as recharging points"""
        ugv_time_windows = ([(int(uav_strt_time), int(uav_strt_time+60))] +  # 0 (start)
                            [(0, veh_max_time+1000)] * n_depot_1 +  # 1-6
                            [(nw_stop_tw_1, veh_max_time)] * n_stop_1 +  # 7-12
                            [(0, veh_max_time+1000)] * (n_depot_2 - 1) +  # 13-17
                            [(nw_stop_tw_1, veh_max_time)] +  # 18
                            [(0, veh_max_time)] * total_mission_pts
                            )
        print("The UGV travel time is: {}".format(nw_stop_tw_1))
        print(ugv_time_windows)
    else:
        veh_max_time = int(nw_stop_tw_1 + 120)
        """First version: Where just two UGV points are considered as recharging points"""
        ugv_time_windows = ([(int(uav_strt_time), int(uav_strt_time+60))] +  # 0 (start)
                            [(0, veh_max_time+1000)] * n_depot_1 +  # 1-6
                            [(nw_stop_tw_1, nw_stop_tw_2)] * n_stop_1 +  # 7-12
                            [(0, veh_max_time+1000)] * (n_depot_2 - 1) +  # 13-17
                            [(nw_stop_tw_1, veh_max_time)] +  # 18
                            [(0, veh_max_time)] * total_mission_pts
                            )
        print("The UGV travel time is: {}".format(nw_stop_tw_1))
        print(ugv_time_windows)

    data['time_windows'] = ugv_time_windows

    data['counter'] = ([0] +  # 0 (start)
                       [0] * (n_sum_rch_stops - 1) +  # 1-17 (stations)
                       [0] +  # 18 (end)
                       [1] * total_mission_pts)
    # print(len(data["counter"]))
    # print(sum(data["counter"]))
    data["start_loc"] = strt_pt
    data["end_loc"] = stp1_ugv_stop
    data["hyperparam_end_loc"] = sp1_end_point[0]
    data["num_vehicles"] = num_uavs
    data["fuel_capacity"] = fuel_capacity
    data["horizon"] = veh_max_time
    data["vehicle_speed"] = 33
    data["starts"] = [0]*num_uavs
    data["ends"] = [n_sum_rch_stops]*num_uavs
    print('End node is {}'.format(data["ends"]))
    data["n_sum_rch_stops"] = n_sum_rch_stops
    data["stations"] = [i for i in range(1, n_sum_rch_stops)]
    distance_matrix = np.zeros((data["num_locations"], data["num_locations"]), dtype=int)
    velocity_ratio = data["vehicle_speed"] // ugv_velocity
    for i in range(data["num_locations"]):
        for j in range(data["num_locations"]):
            if i == j:
                distance_matrix[i][j] = 0
            else:
                distance_matrix[i][j] = euclidean_distance(data["locations"][i], data["locations"][j])
    data["visits"] = [i for i in range(n_sum_rch_stops+1, len(_locations))]
    dist_matrix = distance_matrix.tolist()
    data["distance_matrix"] = dist_matrix
    assert len(data['distance_matrix']) == data['num_locations']
    assert len(data['distance_matrix']) == len(data['time_windows'])
    assert len(data['starts']) == len(data['ends'])
    assert data['num_vehicles'] == len(data['starts'])
    assert len(data["counter"]) == len(data['time_windows'])
    print(data["stations"])
    print(data["visits"])
    time_windows_dict = {}
    for i, time_ in enumerate(data["time_windows"]):
        time_windows_dict[i] = time_
    return data, veh_max_time, ugv_velocity, ugv_velocity, time_windows_dict, _locs_depotb


def euclidean_distance(position_1, position_2):  # This function calculates the distance UAV travels from one mission point to another
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])))


def print_solution(data, manager, routing, solution):  # Prints the UAV optimization solution output on the console.
    # print("Objective: {}".format(solution.ObjectiveValue()))
    time_var = 0
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
    coord_list2 = [(data["coordinates"][0][0], data["coordinates"][0][1])]
    time_elapsed = []
    time_elapsed2 = []
    fuel_remaining = []
    disttravel_track = []
    uav_time_each_veh = []
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
            plan_output += "Location: {0} with Fuel({1}) at Time({2},{3})secs. Slack({4}) FuelSlack({5},{6}) with Distance({7})ft. -> ".format(
                data["coordinates"][manager.IndexToNode(index)],
                data["fuel_capacity"] - solution.Value(fuel_var),
                solution.Min(time_var), solution.Max(time_var),
                solution.Value(slack_var), solution.Min(fuel_slack), solution.Max(fuel_slack), solution.Value(dist_var))
            if vehicle_id == 0:
                time_elapsed.append(round((solution.Min(time_var))/60))
            elif vehicle_id == 1:
                time_elapsed2.append(round((solution.Min(time_var))/60))
            fuel_remaining.append(solution.Value(fuel_var))
            disttravel_track.append(solution.Value(dist_var))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if vehicle_id == 0:
                if index <= len(data["locations"])-1:
                    if index in [i for i in range(data["ends"][0])]:
                        dum_list.append(data["locations"][index])
                        coord_list.append(data["coordinates"][index])
                    else:
                        dum_list.append(data["locations"][manager.IndexToNode(index)])
                        coord_list.append(data["coordinates"][manager.IndexToNode(index)])
            else:
                if index <= len(data["locations"])-1:
                    if index in [i for i in range(data["ends"][0])]:
                        dum_list2.append(data["locations"][index])
                        coord_list2.append(data["coordinates"][index])
                    else:
                        dum_list2.append(data["locations"][index + 1])
                        coord_list2.append(data["coordinates"][index + 1])
        dist_var = distance_dimension.CumulVar(index)
        fuel_var = fuel_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += "Location: {0} with Fuel({1}) at Time({2},{3})secs. with Distance({4})ft.\n".format(
            data["coordinates"][manager.IndexToNode(index)],
            data["fuel_capacity"] - solution.Value(fuel_var),
            solution.Min(time_var), solution.Max(time_var),
            solution.Value(dist_var))
        if vehicle_id == 0:
            time_elapsed.append(round((solution.Min(time_var))/60))
        elif vehicle_id == 1:
            time_elapsed2.append(round((solution.Min(time_var))/60))
        fuel_remaining.append(solution.Value(fuel_var))
        disttravel_track.append(solution.Value(dist_var))
        plan_output += "Distance of the route: {} ft\n".format(solution.Value(dist_var))
        plan_output += "Remaining Fuel of the route: {}\n".format(data["fuel_capacity"] - solution.Value(fuel_var))
        plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
        print(plan_output)
        uav_time_each_veh.append(solution.Value(time_var))
        total_distance += solution.Value(dist_var)
        total_fuel += data['fuel_capacity'] - solution.Value(fuel_var)
        total_time += solution.Value(time_var)
    print('Total Distance of all routes: {} ft'.format(total_distance))
    print('Total Fuel remaining of all routes: {}'.format(total_fuel))
    print('Total Time of all routes: {} seconds'.format(total_time))

    if coord_list[-1][1] == coord_list[-2][1]:
        coord_list.pop(-1)
        time_elapsed.pop(-1)

    # dum_list.append((data["locations"][len(data["stations"])][0], data["locations"][len(data["stations"])][1]))
    # dum_list2.append((data["locations"][len(data["stations"])][0], data["locations"][len(data["stations"])][1]))
    # coord_list.append((data["coordinates"][len(data["stations"])][0], data["coordinates"][len(data["stations"])][1]))
    # coord_list2.append((data["coordinates"][len(data["stations"])][0], data["coordinates"][len(data["stations"])][1]))
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    for i in dum_list:
        x1.append(i[0])
        y1.append(i[1])
    for j in dum_list2:
        x2.append(j[0])
        y2.append(j[1])
    df2_no_mid = pd.DataFrame(coord_list, columns=['x', 'y'])
    df2_no_mid.insert(0, 'time', time_elapsed)
    if data["end_loc"] != data["hyperparam_end_loc"]:
        df2_no_mid.to_excel('Sub_problem1_scenario2_ensemble_1.xlsx', index=False, engine='openpyxl')
    else:
        df2_no_mid.to_excel('Sub_problem1_scenario2_ensemble_2.xlsx', index=False, engine='openpyxl')
    return solution.ObjectiveValue(), total_distance, max(uav_time_each_veh), disttravel_track


def get_routes(data, solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    route = []
    ugv_speed = 10  # in ft/s
    time_dimension = routing.GetDimensionOrDie("Time")
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        time_var_strt = time_dimension.CumulVar(index)
        route.append((data["coordinates"][manager.IndexToNode(index)], solution.Min(time_var_strt)))
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            node = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            if type(data["coordinates"][node][0]) == float:
                route.append((data["coordinates"][node], solution.Min(time_var)))
            else:
                dist_btw_nodes = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(data["coordinates"][node][0], data["coordinates"][node][1], ordered_pts_from_depot)
                time_btw_nodes = int(dist_btw_nodes / ugv_speed)
                route.append((data["coordinates"][node][0], solution.Min(time_var)))
                route.append((data["coordinates"][node][1], solution.Min(time_var)+time_btw_nodes))
            # else:
            #     dist_btw_nodes = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(data["coordinates"][node][0], data["coordinates"][node][1])
            #     time_btw_nodes = int(dist_btw_nodes / ugv_speed)
            #     route.append((data["coordinates"][node][0], solution.Min(time_var), solution.Value(slack_var)))
            #     route.append((data["coordinates"][node][1], solution.Min(time_var)+time_btw_nodes, solution.Value(slack_var)))
        routes.append(route)
    return route, routes


def routes_to_api(data, solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    route = []
    ugv_speed = 10  # in ft/s
    time_dimension = routing.GetDimensionOrDie("Time")
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        time_var_strt = time_dimension.CumulVar(index)
        route.append((data["coordinates"][manager.IndexToNode(index)], solution.Min(time_var_strt)//60))
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            node = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            route.append((data["coordinates"][node], solution.Min(time_var)//60))
        routes.append(route)
    return route, routes


class SolutionCallback(object):
    def __init__(self, model, MaxNoChange, ProgressIncrement):
        self.model = model
        self.MaxNoChange = MaxNoChange
        self.ProgressIncrement = ProgressIncrement
        self.start_time = time()
        self.solution_count = 0
        self.solution_count_best = 0
        self.LastTime = 0
        self.LastBestTime = 0
        self.BestObj = sys.maxsize
        print(" Time  Solution    Objective         Best")
        print("-----------------------------------------")

    def __call__(self):
        global time_limit
        global time_limit_counter
        self.solution_count += 1
        CurrObj = self.model.CostVar().Max()
        CurrTime = time()

        # if (self.BestObj <= CurrObj) and (self.solution_count - self.solution_count_best > self.MaxNoChange):
        #     print(f"Solution best count: {self.solution_count_best}")
        #     print(f"Solution current count: {self.solution_count}")
        if (self.BestObj <= CurrObj) and (CurrTime - self.LastBestTime > self.MaxNoChange):
            print(f"{int(CurrTime - self.start_time):>5,} {self.solution_count:>9,} {CurrObj:>12,} {self.BestObj:>12,}")
            print(f"\nStopped due to no improvement for {CurrTime - self.LastBestTime:.0f} seconds\n")
            self.model.solver().FinishCurrentSearch()
        else:
            if CurrObj < self.BestObj:
                self.BestObj = CurrObj
                self.LastBestTime = CurrTime
                self.solution_count_best = self.solution_count
            if (CurrTime - self.LastTime) >= self.ProgressIncrement and (self.BestObj < sys.maxsize):
                print(f"{int(CurrTime - self.start_time):>5,} {self.solution_count:>9,} {CurrObj:>12,} {self.BestObj:>12,}")
                self.LastTime = CurrTime
        # if CurrTime - self.LastBestTime >= time_limit:
        #     time_limit_counter += 1


def main(strt_pt, sp1_end_point, stp1_ugv_stop, ugv_stops_dict, visited_missions, num_uavs, data_flag):  # This is where optimization of UAV routes happen.
    """Entry point of the program."""
    # Instantiate the data problem.
    # [START data]
    print(f'------------------------------------------------Subproblem 1-------------------------------------------')
    print(f'The corresponding UGV hyperparameters are: Stop1: {stp1_ugv_stop}')
    data, veh_max_time, depotb_vel, ugv_vel, tw_dict, ugv_inter_locs = create_data_model(strt_pt, sp1_end_point, stp1_ugv_stop, ugv_stops_dict, visited_missions, num_uavs, data_flag, ordered_pts_rest)
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
    MaxNoChange = 1.2  # 1000 # iterations # 1.2  # seconds
    ProgressIncrement = 0.1
    time_limit = 5  # seconds
    time_limit_counter = 0
    solution_graph = list()
    start_time = time()

    # def solution_callback():
    #     t = time() - start_time
    #     solution_graph.append((t, routing.CostVar().Max()))
    # routing.AddAtSolutionCallback(solution_callback)

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
    # distance_dimension.SetGlobalSpanCostCoefficient(10)

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
        routing.solver().Add(time_dimension.SlackVar(routing.Start(vehicle_id)) <= 300)
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
                    routing.solver().Add(time_dimension.SlackVar(to_index) <= 300)

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
    search_parameters.time_limit.FromSeconds(3)

    callback = SolutionCallback(routing, MaxNoChange, ProgressIncrement)  # Before call to routing solver
    routing.AddAtSolutionCallback(callback)

    # Solve the problem.
    # [START solve]
    solution = routing.SolveWithParameters(search_parameters)
    routing.AddAtSolutionCallback(fuel_callback)
    # [END solve]

    # Print solution on console.
    # [START print_solution]
    # ugv_tw_array = np.array(data["time_windows"])
    if solution:
        obj_val, total_dist, total_time, disttravel_track = print_solution(data, manager, routing, solution)
        route_dict, route = get_routes(data, solution, routing, manager)
        if obj_val < 1_000_000:
            # print("Objective value is: {}".format(veh_max_time - total_time))
            return veh_max_time - total_time, veh_max_time, total_dist, total_time, depotb_vel, ugv_vel, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs
        else:
            # print("Objective value is: {}".format(obj_val))
            return obj_val, veh_max_time, total_dist, total_time, depotb_vel, ugv_vel, route_dict, route, tw_dict, disttravel_track, ugv_inter_locs
    else:
        print("*************No solution found************")
        return 0, 0, 0, 0, 15, 15, [((0, 0), 0), ((0, 0), 0)], [0, 0], tw_dict, [0, 0], ugv_inter_locs
    # [END print_solution]
    # print("Solver status:", routing.status())
    # print("****************************************************\n")
