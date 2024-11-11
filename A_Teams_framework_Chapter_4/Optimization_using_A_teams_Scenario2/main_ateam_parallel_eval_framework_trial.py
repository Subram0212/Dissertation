import numpy as np
import pandas as pd
import scenario1_genetic_algorithm as ga
import scenario1_innerloop_opt_for_improver_agent
import scenario1_innerloop_opt
import random
from skopt.space import Space
from skopt.sampler import Lhs
import csv
import copy
from scipy.optimize import fmin, fmin_tnc, minimize
from threading import Thread
from queue import Queue
import time
from itertools import product, combinations
import scenario_1_data_processing
import scenario1_innerloop_opt_updated_solution_trial
import scenario1_innerloop_opt_for_improver_agent_updated_solution_trial


def cb(X):
    global Nfeval
    global fout
    fout.write('At iterate {0:4d}, X1={1} X2={2} X3={3}'.format(Nfeval, X[0], X[1], X[2]) + '\n')
    Nfeval += 1


def unique(X_dict):
    X_list = []
    for key, val in X_dict.items():
        X_list.append(key)
    x = np.array(X_list)
    loc_list = []
    for i in x:
        loc_list.append(int(i[0]))
    unique_loc_id = np.unique(loc_list, return_index=True)[1]
    unique_loc = [loc_list[index] for index in sorted(unique_loc_id)]
    unique_x = []
    unique_x_dict = {}
    for j in sorted(unique_loc_id):
        if len(unique_x_dict) < len(unique_loc):
            j = tuple(X_list[j])
            unique_x.append(j)
            unique_x_dict[j] = X_dict[j]
    return unique_x_dict


def local_minimum(X, ugv_stops_dict, ugv_stops_distance_dict, improvement_process, add_ugv_stop_pt):
    minimum = minimize(fun=scenario1_innerloop_opt_for_improver_agent_updated_solution_trial.main, x0=X, args=(ugv_stops_dict, ugv_stops_distance_dict, improvement_process, add_ugv_stop_pt), method='Nelder-Mead', callback=cb, options={'maxfev': 20}, tol=0.2)
    # Apply destroyer agent for 1st improver agent (local) -> Get unique solutions
    improvement_process = unique(improvement_process)
    imp_0, imp_1, imp_2 = minimum.x
    for key, val in improvement_process.items():
        key = tuple(key)
        if val > 1_000_000:
            continue
        loc_population_dict[(int(key[0]), int(key[1]), int(key[2]))] = int(val)

    imp_fitness_val, imp_max_time, imp_total_distance, imp_total_time, imp_depotb_vel, imp_ugv_vel, imp_penalty, imp_route_dict, imp_route, imp_tw_dict = scenario1_innerloop_opt_updated_solution_trial.main([int(imp_0), int(imp_1), int(imp_2)], ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt)
    # population_dict[tuple([int(imp_0), int(imp_1), int(imp_2)])] = imp_fitness_val

    if imp_fitness_val < 6_000_000:
        wt_time_1 = int(imp_1) * 60
        wt_time_2 = int(imp_2) * 60
        count_stp1 = 0
        count_stp2 = 0
        cnt_list_stop1 = []
        cnt_list_stop2 = []
        count_1 = 0
        count_2 = 0
        for key in imp_route_dict:
            if key in [7, 8, 9, 10, 11, 12]:
                count_stp1 += 1
                cnt_list_stop1.append(count_stp1)
            elif key in [13, 14, 15, 16, 17, 18]:
                count_stp2 += 1
                cnt_list_stop2.append(count_stp2)
        for key in imp_route_dict:
            if key in [7, 8, 9, 10, 11, 12]:
                count_1 += 1
                if imp_route_dict[key] < imp_tw_dict[key][1] and count_1 == max(cnt_list_stop1):
                    wt_time_1 -= (imp_tw_dict[key][1] - imp_route_dict[key])
                    wt_time_1 = (wt_time_1 + 180) // 60
            elif key in [13, 14, 15, 16, 17, 18]:
                count_2 += 1
                if imp_route_dict[key] < imp_tw_dict[key][1] and count_2 == max(cnt_list_stop2):
                    wt_time_2 -= (imp_tw_dict[key][1] - imp_route_dict[key])
                    wt_time_2 = (wt_time_2 + 180) // 60
        # if imp_stop2:
        #     wt_time_2 = 180 // 60
        # if imp_stop1:
        #     wt_time_1 = 180 // 60
        imp_1 = wt_time_1
        imp_2 = wt_time_2
        proper_fitness_val, proper_max_time, proper_total_distance, proper_total_time, proper_depotb_vel, proper_ugv_vel, proper_penalty, proper_route_dict, proper_route, proper_tw_dict, stp1s, stp2s = scenario1_innerloop_opt_updated_solution_trial.main([imp_0, imp_1, imp_2], ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt)
        uav_arr_time = proper_route_dict[23]
        ugv_depart_time = proper_tw_dict[23][1]
        ugv_travel_distance = (uav_arr_time - ugv_depart_time) * ugv_vel  # ugv should travel this distance to catch up to the UAV point
        if ugv_stops_dict[param_list[0]][0] in stp1s:
            for i in range(len(ugv_route_pts)):
                if ((ugv_stops_distance_dict[(ugv_stops_dict[param_list[0]][0], ugv_route_pts[i])]) - (ugv_travel_distance / 5280)) <= 0.3:
                    add_ugv_stop_pt = ugv_route_pts[i]
                    break
        elif ugv_stops_dict[param_list[0]][1] in stp1s:
            for i in range(len(ugv_route_pts)):
                if ((ugv_stops_distance_dict[(ugv_stops_dict[param_list[0]][1], ugv_route_pts[i])]) - (ugv_travel_distance / 5280)) <= 0.3:
                    add_ugv_stop_pt = ugv_route_pts[i]
                    break
        final_fitness_val, final_max_time, final_total_distance, final_total_time, final_depotb_vel, final_ugv_vel, final_penalty, final_route_dict, final_route, \
        final_tw_dict, stp1s, stp2s = scenario1_innerloop_opt_updated_solution_trial.main(param_list, ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt)
        if penalty > 0:
            final_fitness_val += penalty
        if final_fitness_val == 0 or total_distance == 0 or total_time == 0:
            final_fitness_val = 50_000_000
        loc_population_dict[(int(imp_0), int(imp_1), int(imp_2))] = final_fitness_val
        print("Local population: ", loc_population_dict)
    return loc_population_dict


def global_minimum(X_list, ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt):
    evolution_gen = ga.operator_selection(X_list)
    for sol in evolution_gen:
        global_f, global_t, global_d, global_tt, global_depotvel, global_ugvvel, global_pen, global_rd, global_r, global_tw_dict, stp1s, stp2s = scenario1_innerloop_opt_updated_solution_trial.main([sol[0], sol[1], sol[2]], ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt)
        # Apply destroyer agent for 2nd improver agent (global): -> One agent works on the intermediate results of another.
        if global_f > 1_000_000:
            continue
        glob_population_dict[tuple(sol)] = global_f
    print("Global population: ", glob_population_dict)
    return glob_population_dict


combo = []


def ugv_distance_calc():
    ugv_stops_distance_dict = {}
    df = pd.read_csv('Scenario 1 data points.csv')
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
        ugv_stops_distance_dict[combo[j]] = scenario_1_data_processing.ugv_distance(combo[j][0], combo[j][1])
    return ugv_stops_distance_dict
    # print(ugv_stops_distance_dict)

# dum_list = {tuple([58, 25, 10]): 2354, tuple([56, 25, 10]): 2354, tuple([55, 26, 9]): 2354, tuple([54, 24, 10]): 2354, tuple([55, 25, 10]): 2354, tuple([56, 26, 9]): 2354, tuple([56, 26, 9]): 2354, tuple([56, 26, 9]): 2354}
# unique_dict = unique(dum_list)
# Agent descriptions

"""Constructor components:
    Constructor requestors: Request to run only once or runs until a feasible solution is obtained.
    Constructors use null selectors.
    Operators: The algorithm for solution constructor: -> In this case it is random LHS sampling.
    Distributor: Adds the solution to the population. In case of constructor, it is just the random solutions obtained
    from sampling.
"""

t0 = time.time()
np.random.seed(1)
population_dict = {}
loc_population_dict = {}
glob_population_dict = {}
overall_population_dict = []
n_samples = 60
space = Space([(2, 141), (120//60, 3000//60), (120//60, 3000//60)])  # try having only one depot location - dont forget to add multi depot afterwards
lhs = Lhs(lhs_type="classic", criterion=None)
x = lhs.generate(space.dimensions, n_samples)
# for l in range(len(x)):
#     x[l][3] = 5

# Consolidating possible UGV stop location pairs into a hash table (dictionary)
ugv_stops_dict = scenario_1_data_processing.scenario_1_data_processing()
ugv_stops_distance_dict = ugv_distance_calc()

x_copy = copy.deepcopy(x)
ugv_route_pts = [(4.09, 0.96), (4.21, 1.44), (4.33, 1.91),
                 (4.45, 2.39), (4.57, 2.86), (4.69, 3.34), (3.97, 0.49),
                 (4.28, 0.74), (4.59, 0.99), (4.9, 1.25), (5.21, 1.5), (5.51, 1.75), (5.82, 2.01),
                 (6.13, 2.26)]
add_ugv_stop_pt = random.choice(ugv_route_pts)
switching_phase_solution = []
sorted_initial_population = x
print(x)
duration_dict = {0: ['NW_stop', 'SE_stop', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps', 'start point position']}
for n in range(len(sorted_initial_population)):
    param_list = sorted_initial_population.pop(0)
    duration_dict[n+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
    fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, tw_dict, disttravel_track = scenario1_innerloop_opt.main(param_list, ugv_stops_dict, ugv_stops_distance_dict)
    if penalty > 0:
        fitness_val += penalty
    if fitness_val == 0 or total_distance == 0 or total_time == 0:
        fitness_val = 50_000_000
    while 198*((disttravel_track[-1] - disttravel_track[-2]) // 33) > 287700:
        proper_fitness_val, proper_max_time, proper_total_distance, proper_total_time, proper_depotb_vel, proper_ugv_vel, proper_penalty, proper_route_dict, proper_route, \
        proper_tw_dict, stp1s, stp2s = scenario1_innerloop_opt_updated_solution_trial.main(param_list, ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt)
        if ugv_stops_dict[param_list[0]][0] in stp1s:
            uav_arr_time = proper_route_dict[23]
            ugv_depart_time = proper_tw_dict[10][1]
            ugv_travel_distance = (uav_arr_time - ugv_depart_time) * ugv_vel  # ugv should travel this distance to catch up to the UAV point
            for i in range(len(ugv_route_pts)):
                if abs((ugv_stops_distance_dict[(ugv_stops_dict[param_list[0]][0], ugv_route_pts[i])]) - (ugv_travel_distance / 5280)) <= 0.1:
                    add_ugv_stop_pt = ugv_route_pts[i]
                    break
        elif ugv_stops_dict[param_list[0]][1] in stp1s:
            uav_arr_time = proper_route_dict[23]
            ugv_depart_time = proper_tw_dict[16][1]
            ugv_travel_distance = (uav_arr_time - ugv_depart_time) * ugv_vel  # ugv should travel this distance to catch up to the UAV point
            for i in range(len(ugv_route_pts)):
                if abs((ugv_stops_distance_dict[(ugv_stops_dict[param_list[0]][1], ugv_route_pts[i])]) - (ugv_travel_distance / 5280)) <= 0.1 or abs((ugv_stops_distance_dict[( ugv_route_pts[i], ugv_stops_dict[param_list[0]][1])]) - (ugv_travel_distance / 5280)) <= 0.1:
                    add_ugv_stop_pt = ugv_route_pts[i]
                    break
        proper_fitness_val, proper_max_time, proper_total_distance, proper_total_time, proper_depotb_vel, proper_ugv_vel, proper_penalty, proper_route_dict, proper_route, \
        proper_tw_dict, stp1s, stp2s = scenario1_innerloop_opt_updated_solution_trial.main(param_list, ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt)
    population_dict[tuple(param_list)] = fitness_val
    # if fitness_val < 1_000_000:
    #     wt_time_1 = param_list[1] * 60
    #     wt_time_2 = param_list[2] * 60
    #     count_stp1 = 0
    #     count_stp2 = 0
    #     cnt_list_stop1 = []
    #     cnt_list_stop2 = []
    #     count_1 = 0
    #     count_2 = 0
    #     for key in route_dict:
    #         if key in [7, 8, 9, 10, 11, 12]:
    #             count_stp1 += 1
    #             cnt_list_stop1.append(count_stp1)
    #         elif key in [13, 14, 15, 16, 17, 18]:
    #             count_stp2 += 1
    #             cnt_list_stop2.append(count_stp2)
    #     for key in route_dict:
    #         if key in [7, 8, 9, 10, 11, 12]:
    #             count_1 += 1
    #             if route_dict[key] < tw_dict[key][1] and count_1 == max(cnt_list_stop1):
    #                 wt_time_1 -= (tw_dict[key][1] - route_dict[key])
    #                 wt_time_1 = (wt_time_1 + 180) // 60
    #         elif key in [13, 14, 15, 16, 17, 18]:
    #             count_2 += 1
    #             if route_dict[key] < tw_dict[key][1] and count_2 == max(cnt_list_stop2):
    #                 wt_time_2 -= (tw_dict[key][1] - route_dict[key])
    #                 wt_time_2 = (wt_time_2 + 180) // 60
    #     if stop2:
    #         wt_time_2 = 180 // 60
    #     if stop1:
    #         wt_time_1 = 180 // 60
    #     param_list[1] = wt_time_1
    #     param_list[2] = wt_time_2
    #     fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = inner_loop_optimization.main(param_list)
    #     # if penalty > 0:
    #     #     fitness_val += penalty
    #     # if fitness_val == 0 or total_distance == 0 or total_time == 0:
    #     #     fitness_val = 50_000_000
    if fitness_val <= 2_000_000:  # Even though initial pop is randomized, this framework allows to perform local min even if the solution is infeasible but close to feasibility.
        switching_phase_solution.append(param_list)
        if len(population_dict) <= 2:
            continue
        else:
            break
# Switch from constructor phase to improver phase
"""Improver components:
Improver requestor: Requests to start running the improver agents and the improver filters request to run 
solutions that are feasible and filters the other infeasible solutions.
Improver selectors: Modify the solutions for improvement.
Improver operator: Local and global optimization algorithms
Improver distributor: Adds improved solutions to output population 'population_dict'.
"""
# Initially apply local minimization algorithm (Improver agent):


def wrapper1(func, arg1, arg2, arg3, arg4, arg5, queue):
    queue.put(func(arg1, arg2, arg3, arg4, arg5))


def wrapper2(func, arg, arg1, arg2, arg3, queue):
    queue.put(func(arg, arg1, arg2, arg3))


improvement_process = {}
q1 = Queue()
q2 = Queue()
flag = 0
for key, val in population_dict.items():
    if val > 6_000_000 or len(population_dict) == 1:
        flag = 1
        break
while flag == 1:
    Nfeval = 1
    fout = open('NM_steps_'+'.txt', 'w')
    switch_sol_array = np.asarray(switching_phase_solution.pop())
    count = 0
    for key, val in population_dict.items():
        if val > 1_000_000 or len(population_dict) == 1:
            count += 1
    if count == 0:
        flag = 0
        break
    evolution_with_obj_val = []
    for i, j in population_dict.items():
        evolution_with_obj_val.append([i[0], i[1], i[2], population_dict[i]])

    # Apply local and global optimization algorithm asynchronously/in parallel on the selected solutions (Improver agent):
    evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[3]))
    Thread(target=wrapper1, args=(local_minimum, switch_sol_array, ugv_stops_dict, ugv_stops_distance_dict, improvement_process, add_ugv_stop_pt, q1)).start()
    Thread(target=wrapper2, args=(global_minimum, evolution_with_obj_val, ugv_stops_dict, ugv_stops_distance_dict, add_ugv_stop_pt, q2)).start()
    loc_population_dict = q1.get()
    glob_population_dict = q2.get()
    for key, val in loc_population_dict.items():
        population_dict[key] = val
    for key, val in glob_population_dict.items():
        population_dict[key] = val
    count = 0
    population_dict = dict(sorted(population_dict.items(), key=lambda item: item[1]))
    overall_population_dict.append(population_dict)
    for key, val in population_dict.items():
        if val > 1_000_000:
            count += 1
    if count == 0:
        flag = 0
    if len(overall_population_dict) > 1:
        dict_count = 0
        last_dict = overall_population_dict[-1]
        last_before_dict = overall_population_dict[-2]
        for key in last_dict:
            if key in last_before_dict and last_dict[key] == last_before_dict[key]:
                dict_count += 1
        if dict_count == len(population_dict):
            flag = 0
    if len(population_dict) > 30:
        for _ in range(len(population_dict) - 30):
            population_dict.popitem()
    print(population_dict)
    itr = 0
    for key, val in population_dict.items():
        if itr == 0:
            switching_phase_solution.append([list(key)])
            itr += 1
            break

t1 = time.time() - t0
print("Elapsed time = {}".format(t1))
