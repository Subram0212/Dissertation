import numpy as np
import pandas as pd
import genetic_algorithm as ga
import inner_loop_optimization_for_improver
import inner_loop_optimization
import local_search
import random
from skopt.space import Space
from skopt.sampler import Lhs
import csv
import copy
from scipy.optimize import fmin, fmin_tnc, minimize
import time


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
np.random.seed(5)
population_dict = {}
loc_population_dict = {}
glob_population_dict = {}
overall_population_dict = []
n_samples = 60
space = Space([(2, 58), (120//60, 3000//60), (120//60, 3000//60)])  # try having only one depot location - dont forget to add multi depot afterwards
lhs = Lhs(lhs_type="classic", criterion=None)
x = lhs.generate(space.dimensions, n_samples)
# for l in range(len(x)):
#     x[l][3] = 2
print(x)

# Consolidating possible UGV stop location pairs into a hash table (dictionary)
ugv_stops_dict = {}
# ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)], [(3.1, 7.24), (10.44, 0.96)], [(3.1, 7.24), (10.54, 0.9)],
#              [(2.77, 7.57), (7.7, 3.61)], [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.77, 7.57), (10.44, 0.96)], [(2.77, 7.57), (10.54, 0.9)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.43, 7.91), (10.44, 0.96)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (7.45, 3.86)],
#              [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (8.39, 2.98)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(2.61, 8.34), (10.44, 0.96)], [(2.61, 8.34), (10.54, 0.9)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)], [(3.01, 8.87), (10.44, 0.96)], [(3.01, 8.87), (10.54, 0.9)],
#              [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.29, 9.23), (9.53, 2.29)], [(3.29, 9.23), (10.44, 0.96)], [(3.29, 9.23), (10.54, 0.9)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.42, 9.61), (10.44, 0.96)], [(3.42, 9.61), (10.54, 0.9)],
#              [(3.74, 10.45), (10.44, 0.96)], [(3.74, 10.45), (10.54, 0.9)], [(3.54, 10.04), (10.44, 0.96)], [(3.54, 10.04), (10.54, 0.9)]]

ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)],
             [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)], [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)],
             [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)], [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)],
             [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)], [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)],
             [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)],
             [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)], [(3.42, 9.61), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)], [(3.54, 10.04), (10.54, 0.9)], [(3.74, 10.45), (10.54, 0.9)]]

# rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
# # for r in range(len(ugv_Stops)):
# #     rev_ugv_stops_lst.append(ugv_Stops[r])
#
# for i in range(len(rev_ugv_stops_lst)):
#     rev_ugv_stops_lst[i].reverse()
#
# for l in range(len(rev_ugv_stops_lst)):
#     ugv_Stops.append(rev_ugv_stops_lst[l])

# print(ugv_Stops)

for i in range(len(ugv_Stops)):
    ugv_stops_dict[i+2] = ugv_Stops[i]

x_copy = copy.deepcopy(x)

switching_phase_solution = []
sorted_initial_population = x
print(x)
duration_dict = {0: ['NW_stop', 'SE_stop', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps', 'start point position']}
for n in range(len(sorted_initial_population)):
    param_list = sorted_initial_population.pop(0)
    duration_dict[n+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
    fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = inner_loop_optimization.main(param_list)
    if penalty > 0:
        fitness_val += penalty
    if fitness_val == 0 or total_distance == 0 or total_time == 0:
        fitness_val = 50_000_000
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
    if fitness_val < 1_000_000 or fitness_val <= 2_000_000 or param_list[0] >= 50:  # Even though initial pop is randomized, this framework allows to perform local min even if the solution is infeasible but close to feasibility.
        switching_phase_solution.append(param_list)
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
improvement_process = {}

flag = 0
for key, val in population_dict.items():
    if val > 6_000_000:
        flag = 1
        break
while flag == 1:
    Nfeval = 1
    fout = open('NM_steps_'+'.txt', 'w')
    switch_sol_array = np.asarray(switching_phase_solution.pop())
    count = 0
    for key, val in population_dict.items():
        if val > 1_000_000:
            count += 1
    if count == 0:
        flag = 0
        break
    minimum = minimize(fun=inner_loop_optimization_for_improver.main, x0=switch_sol_array, args=improvement_process, method='Nelder-Mead', callback=cb, options={'maxfev': 20}, tol=0.2)
    # Apply destroyer agent for 1st improver agent (local) -> Get unique solutions
    improvement_process = unique(improvement_process)
    imp_0, imp_1, imp_2 = minimum.x
    for key, val in improvement_process.items():
        key = tuple(key)
        if val > 1_000_000:
            continue
        loc_population_dict[(int(key[0]), int(key[1]), int(key[2]))] = int(val)

    imp_fitness_val, imp_max_time, imp_total_distance, imp_total_time, imp_depotb_vel, imp_ugv_vel, imp_penalty, imp_route_dict, imp_route, imp_stop1, imp_stop2, imp_tw_dict = inner_loop_optimization.main([int(imp_0), int(imp_1), int(imp_2)])
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
        if imp_stop2:
            wt_time_2 = 180 // 60
        if imp_stop1:
            wt_time_1 = 180 // 60
        imp_1 = wt_time_1
        imp_2 = wt_time_2
        fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = inner_loop_optimization.main([imp_0, imp_1, imp_2])
        if penalty > 0:
            fitness_val += penalty
        if fitness_val == 0 or total_distance == 0 or total_time == 0:
            fitness_val = 50_000_000
        loc_population_dict[(int(imp_0), int(imp_1), int(imp_2))] = fitness_val
        print(loc_population_dict)


    # Apply global optimization algorithm on the selected solutions (Improver agent):
    evolution_with_obj_val = []
    for i, j in population_dict.items():
        evolution_with_obj_val.append([i[0], i[1], i[2], population_dict[i]])
    evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[3]))
    evolution_gen = ga.operator_selection(evolution_with_obj_val)
    for sol in evolution_gen:
        global_f, global_t, global_d, global_tt, global_depotvel, global_ugvvel, global_pen, global_rd, global_r, global_stop1, global_stop2, global_tw_dict = inner_loop_optimization.main([sol[0], sol[1], sol[2]])
        # Apply destroyer agent: -> One agent works on the intermediate results of another.
        if global_f > 1_000_000:
            continue
        glob_population_dict[tuple(sol)] = global_f
    print(glob_population_dict)
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
