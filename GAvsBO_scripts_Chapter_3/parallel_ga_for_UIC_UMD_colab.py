"""This code is optimizing the UGV parameters using Genetic algorithm
Functions applied in this Genetic Algorithm code are described as follows:

binary_to_decimal: This function decodes the chromosomes (binary encoding) into decimal values of the UGV parameters.
decimal_to_binary: This function encodes the UGV parameters into binary numbers.
operator_selection: This function performs the Selection, crossover and mutation operations of the Genetic Algorithm Each operation has specific function.
tournament_selection: This function performs the Tournament selection operation.
mate: This function performs the crossover and mutation operations.
bitstring_to_param_list: This function separates the chromosomes into appropriate UGV parameters that can be understood by the inner-loop optimization code."""

import numpy as np
import UIC_UMD_collab_innerloop_opt
import random
from skopt.space import Space
from skopt.sampler import Lhs
import csv
import pandas as pd
import copy
import time
from threading import Thread
from queue import Queue


def binary_to_decimal(param_list):  # This function takes UGV parameters as arguments
    num = []
    dec_value = []
    for i in range(len(param_list)):
        num.append(param_list[i])
        dec_value.append(0)

        # Initializing base
        # value to 1, i.e 2 ^ 0
        base = 1
        temp = int(num[i])
        while(temp):
            last_digit = temp % 10
            temp = int(temp / 10)

            dec_value[i] += last_digit * base
            base = base * 2
    return dec_value


def decimal_to_binary(param_list):  # This function takes UGV parameters as arguments
    num = []
    bin_value = []
    ordered_str_list = []
    for j in range(len(param_list)):
        num.append(param_list[j])
        bin_value.append(0)

        temp = num[j]
        temp_str = str()
        while temp > 1:
            last_digit = temp % 2
            temp_str += str(last_digit)
            # print(temp_str)
            temp = temp // 2
            if temp == 1 or temp == 0:
                temp_str += str(temp)
        ordered_str = temp_str[::-1]
        if len(ordered_str) < 8:
            diff = 8 - len(ordered_str)
            for i in range(diff):
                res = list(ordered_str)
                res.insert(i, '0')
                ordered_str = ''.join(res)
        ordered_str_list.append(ordered_str)
        bin_value[j] = int(ordered_str)
    return bin_value, ordered_str_list


# def operator_selection(init_population_sorted, evolution_with_obj):  # This function takes current generation population obtained from evolution as input arguments
#     # Selection -> Tournament selection is to be done
#     pop_size = len(init_population_sorted)
#     init_pop_sorted_copy = copy.deepcopy(init_population_sorted)
#     evolution_with_obj_copy = copy.deepcopy(evolution_with_obj)
#     next_generation = []
#     # perform Ellitism: have some chromosomes from the previous generation. For example, 5% of population
#     for i in range(len(init_population_sorted)//2 + 2):
#         print(i)
#         if i < 4:
#             next_generation.append(evolution_with_obj_copy[i])
# #             init_pop_sorted_copy.pop(0)
#         else:
# #             evolution_with_obj_copy.pop(0)
#             father = tournament_selection(evolution_with_obj)
#             mother = tournament_selection(evolution_with_obj)
#             while father == mother:
#                 mother = tournament_selection(evolution_with_obj)
#             print("Father before deletion: {}".format(father))
#             print("Mother before deletion: {}".format(mother))
#             objval_1 = father[3]
#             objval_2 = mother[3]
#             del father[-1]
#             del mother[-1]
#             print("Father after deletion: {}".format(father))
#             print("Mother after deletion: {}".format(mother))
#             # print(father)
#             # print(mother)
#             child1, child2 = mate(father, mother)
#             child1_conv = bitstring_to_param_list(child1)
#             # print(child1_conv)
#             child2_conv = bitstring_to_param_list(child2)
#             # print(child2_conv)
#             child1_conv_decimal = binary_to_decimal(child1_conv)
#             child2_conv_decimal = binary_to_decimal(child2_conv)
#             next_generation.append(child1_conv_decimal)
#             next_generation.append(child2_conv_decimal)
#             print(child1_conv_decimal)
#             print(child2_conv_decimal)
#             father.insert(3, objval_1)
#             mother.insert(3, objval_2)
#     return next_generation

def operator_selection(init_population_sorted, evolution_with_obj):  # This function takes current generation population obtained from evolution as input arguments
    """init_population_sorted: The population of the current generation is sorted and sent as the argument.

       evolution_with_obj: This argument is the same as the init_population_sorted but with an extra objective/fitness val column"""
    # Selection -> Tournament selection is to be done
    pop_size = len(init_population_sorted)
    init_pop_sorted_copy = copy.deepcopy(init_population_sorted)
    evolution_with_obj_copy = copy.deepcopy(evolution_with_obj)
    first_candidate_set = copy.deepcopy(evolution_with_obj)
    next_generation = []
    perm_list = permutation(evolution_with_obj)
    second_list_father = []
    second_list_mother = []
    for i in range(len(evolution_with_obj)):
        if i % 2 == 0:
            second_list_father.append(perm_list[i])
        else:
            second_list_mother.append(perm_list[i])
    # second_list_father = [f for f in perm_list if perm_list.index(f) % 2 == 0]
    # second_list_mother = [m for m in perm_list if perm_list.index(m) % 2 == 1]
    # perform Ellitism: have some chromosomes from the previous generation. For example, 5% of population
    for i in range(len(init_population_sorted)//2 + 2):
        # print(i)
        if i < 4:
            # objval = evolution_with_obj_copy[i][5]
            del evolution_with_obj_copy[i][-1]
            next_generation.append(evolution_with_obj_copy[i])
            # evolution_with_obj_copy.pop(0)
            # evolution_with_obj_copy[i].insert(5, objval)
        else:
            # evolution_with_obj_copy.pop(0)
            father = unbiased_tournament_selection(first_candidate_set, second_list_father)
            mother = unbiased_tournament_selection(first_candidate_set, second_list_mother)
            # while father == mother:
            #     mother = tournament_selection(evolution_with_obj)
            print("Father before deletion: {}".format(father))
            print("Mother before deletion: {}".format(mother))
            third_ele_father1 = father[2]
            third_ele_mother1 = mother[2]
            objval_1 = father[3]
            objval_2 = mother[3]
            father.pop()
            mother.pop()
            # del father[-1]
            # del mother[-1]
            print("Father after deletion: {}".format(father))
            print("Mother after deletion: {}".format(mother))
            if len(father) < 3:
                father.insert(2, third_ele_father1)
            if len(mother) < 3:
                mother.insert(2, third_ele_mother1)
            # print(father)
            # print(mother)
            child1, child2 = mate(father, mother)
            child1_conv = bitstring_to_param_list(child1)
            # print(child1_conv)
            child2_conv = bitstring_to_param_list(child2)
            # print(child2_conv)
            child1_conv_decimal = binary_to_decimal(child1_conv)
            child2_conv_decimal = binary_to_decimal(child2_conv)
            next_generation.append(child1_conv_decimal)
            next_generation.append(child2_conv_decimal)
            print(child1_conv_decimal)
            print(child2_conv_decimal)
            father.insert(3, objval_1)
            mother.insert(3, objval_2)
    return next_generation


def unbiased_tournament_selection(first_set, second_set):  # This function takes current population as input arguments. It is a binary tournament.
    # candidate_1 = random.sample(init_pop_with_obj, 1)
    # candidate_2 = random.sample(init_pop_with_obj, 1)
    candidate_1 = first_set.pop(0)
    candidate_2 = second_set.pop(0)
    # while candidate_1 == candidate_2:
    #     candidate_2 = random.sample(init_pop_with_obj, 1)
    #     candidate_2 = candidate_2.pop(0)
    print("Candidate 1: {}".format(candidate_1))
    print("Candidate 2: {}".format(candidate_2))
    if candidate_1[3] <= candidate_2[3]:
        return candidate_1
    else:
        return candidate_2

# def tournament_selection(init_pop_with_obj):  # This function takes current population as input arguments
#     candidate_1 = random.sample(init_pop_with_obj, 1)
#     candidate_2 = random.sample(init_pop_with_obj, 1)
#     candidate_1 = candidate_1.pop(0)
#     candidate_2 = candidate_2.pop(0)
#     while candidate_1 == candidate_2:
#         candidate_2 = random.sample(init_pop_with_obj, 1)
#         candidate_2 = candidate_2.pop(0)
#     print("Candidate 1: {}".format(candidate_1))
#     print("Candidate 2: {}".format(candidate_2))
#     if candidate_1[3] <= candidate_2[3]:
#         return candidate_1
#     else:
#         return candidate_2


def permutation(evolution):
    perm1_dict = {}
    idx = 0
    for j in range(len(evolution)):
        perm1_dict[j] = evolution[j]
    perm_1l = [i for i in perm1_dict.keys()]
    perm_2 = np.random.permutation(perm_1l)
    perm_2l = perm_2.tolist()
    res = []
    for l in perm_1l:
        if l == perm_2l[idx]:
            res.append(idx)
        idx += 1
    while len(res) > 0:
        idx = 0
        perm_2 = np.random.permutation(perm_1l)
        perm_2l = perm_2.tolist()
        res = []
        for l in perm_1l:
            if l == perm_2l[idx]:
                res.append(idx)
            idx += 1
    perm2_dict = {}
    for k in perm_2l:
        perm2_dict[k] = evolution[k]
    candidate_2_set = [i for i in perm2_dict.values()]
    return candidate_2_set


def mate(father, mother):  # This function performs crossover and mutation and takes two different UGV parameter lists as input arguments.
    bin_father, bin_father_str = decimal_to_binary(father)
    bin_mother, bin_mother_str = decimal_to_binary(mother)
    chromosomes_father = bin_father_str[0]+bin_father_str[1]+bin_father_str[2]
    chromosomes_mother = bin_mother_str[0]+bin_mother_str[1]+bin_mother_str[2]
    print(chromosomes_father)
    print(chromosomes_mother)
    # 2-point Xover operation
    k1 = int(round(random.uniform(1, (len(chromosomes_father)//2)), 0))
    k2 = int(round(random.uniform(k1, len(chromosomes_father)), 0))
    while k1 == k2:
        k2 = int(round(random.uniform(k1, len(chromosomes_father)), 0))
    offspring1 = str()
    offspring2 = str()
    for j in range(len(chromosomes_father)):
        if j < k1:
            offspring1 += chromosomes_father[j]
            offspring2 += chromosomes_mother[j]
        elif k1 <= j < k2:
            offspring1 += chromosomes_mother[j]
            offspring2 += chromosomes_father[j]
        elif j >= k2:
            offspring1 += chromosomes_father[j]
            offspring2 += chromosomes_mother[j]
    # print("Offspring1 before mutation: {}".format(offspring1))
    # print("Offspring2 before mutation: {}".format(offspring2))

    # mutation operation
    p_mutation = 0.01
    for k in range(len(offspring1)):
        if random.random() < p_mutation:
            if offspring1[k] == '0':
                offspring1[k].replace('0', '1')
            else:
                offspring1[k].replace('1', '0')
            if offspring2[k] == '0':
                offspring2[k].replace('0', '1')
            else:
                offspring2[k].replace('1', '0')
    offspring1_list = bitstring_to_param_list(offspring1)
    if int(offspring1_list[0]) > 111010 or int(offspring1_list[0]) < 10:
        if int(offspring1_list[0]) > 111010:
            offspring1_list[0] = '00111010'
        if int(offspring1_list[0]) < 10:
            offspring1_list[0] = '00000010'
    # if int(offspring1_list[1]) > 1111 or int(offspring1_list[1]) < 1110:
    #     if int(offspring1_list[1]) > 1111:
    #         offspring1_list[1] = '00001111'
    #     if int(offspring1_list[1]) < 1110:
    #         offspring1_list[1] = '00001110'
    # if int(offspring1_list[2]) > 1111 or int(offspring1_list[2]) < 1110:
    #     if int(offspring1_list[2]) > 1111:
    #         offspring1_list[2] = '00001111'
    #     if int(offspring1_list[2]) < 1110:
    #         offspring1_list[2] = '00001110'
    if int(offspring1_list[1]) > 110010 or int(offspring1_list[1]) < 10:
        if int(offspring1_list[1]) > 110010:
            offspring1_list[1] = '00110010'
        if int(offspring1_list[1]) < 10:
            offspring1_list[1] = '00000010'
    if int(offspring1_list[2]) > 110010 or int(offspring1_list[2]) < 10:
        if int(offspring1_list[2]) > 110010:
            offspring1_list[2] = '00110010'
        if int(offspring1_list[2]) < 10:
            offspring1_list[2] = '00000010'
    # if int(offspring1_list[5]) > 101 or int(offspring1_list[5]) < 10:
    #     if int(offspring1_list[5]) > 101:
    #         offspring1_list[5] = '00000101'
    #     if int(offspring1_list[5]) < 10:
    #         offspring1_list[5] = '00000010'
    offspring1 = offspring1_list[0]+offspring1_list[1]+offspring1_list[2]
    offspring2_list = bitstring_to_param_list(offspring2)
    if int(offspring2_list[0]) > 111010 or int(offspring2_list[0]) < 10:
        if int(offspring2_list[0]) > 111010:
            offspring2_list[0] = '00111010'
        if int(offspring2_list[0]) < 10:
            offspring2_list[0] = '00000010'
    # if int(offspring2_list[1]) > 1111 or int(offspring2_list[1]) < 1110:
    #     if int(offspring2_list[1]) > 1111:
    #         offspring2_list[1] = '00001111'
    #     if int(offspring2_list[1]) < 1110:
    #         offspring2_list[1] = '00001110'
    # if int(offspring2_list[2]) > 1111 or int(offspring2_list[2]) < 1110:
    #     if int(offspring2_list[2]) > 1111:
    #         offspring2_list[2] = '00001111'
    #     if int(offspring2_list[2]) < 1110:
    #         offspring2_list[2] = '00001110'
    if int(offspring2_list[1]) > 110010 or int(offspring2_list[1]) < 10:
        if int(offspring2_list[1]) > 110010:
            offspring2_list[1] = '00110010'
        if int(offspring2_list[1]) < 10:
            offspring2_list[1] = '00000010'
    if int(offspring2_list[2]) > 110010 or int(offspring2_list[2]) < 10:
        if int(offspring2_list[2]) > 110010:
            offspring2_list[2] = '00110010'
        if int(offspring2_list[2]) < 10:
            offspring2_list[2] = '00000010'
    # if int(offspring2_list[5]) > 101 or int(offspring2_list[5]) < 10:
    #     if int(offspring2_list[5]) > 101:
    #         offspring2_list[5] = '00000101'
    #     if int(offspring2_list[5]) < 10:
    #         offspring2_list[5] = '00000010'
    offspring2 = offspring2_list[0]+offspring2_list[1]+offspring2_list[2]
    print("Offspring1 after mutation: {}".format(offspring1))
    print("Offspring2 after mutation: {}".format(offspring2))
    print(len(offspring1), len(offspring2))
    return offspring1, offspring2


def bitstring_to_param_list(bitstring):  # This function takes an entire chromosome as input argument and returns the appropriate UGV parameter list.
    parameter_list_conv = []
    param_1 = str()
    param_2 = str()
    param_3 = str()
    # param_4 = str()
    # param_5 = str()
    # param_6 = str()
    for i in range(len(bitstring)):
        if i < 8:
            param_1 += bitstring[i]
        elif 8 <= i < 16:
            param_2 += bitstring[i]
        elif 16 <= i < 24:
            param_3 += bitstring[i]
        # elif 24 <= i < 32:
        #     param_4 += bitstring[i]
        # elif 32 <= i < 40:
        #     param_5 += bitstring[i]
        # elif 40 <= i < 48:
        #     param_6 += bitstring[i]i
    parameter_list_conv.append(param_1)
    parameter_list_conv.append(param_2)
    parameter_list_conv.append(param_3)
    # parameter_list_conv.append(param_4)
    # parameter_list_conv.append(param_5)
    # parameter_list_conv.append(param_6)
    return parameter_list_conv


'''param_list_order = [ugv_stop_loc, ugv_depotb_to_ugvstop_velocity, ugv_velocity_between_two_ugvstops, NW_wait_time, SE_wait_time, period_starting_location]'''

'''Step 1: Initial population'''  # Generate using Latin Hypercube Sampling (LHS)


def ga_evaluation1(X):
    sorted_initial_population_copy = copy.deepcopy(X)
    duration_dict = {0: ['NW_stop', 'SE_stop', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps', 'start point position']}
    for j in range(len(sorted_initial_population_copy)):
        param_list = sorted_initial_population_copy.pop(0)
        # print(param_list[5])
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        if penalty > 0:
            fitness_val += penalty
        if fitness_val == 0 or total_distance == 0 or total_time == 0:
            fitness_val = 50_000_000
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
        #     fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        #     if penalty > 0:
        #         fitness_val += penalty
        #     if fitness_val == 0 or total_distance == 0 or total_time == 0:
        #         fitness_val = 50_000_000
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        duration_dict[j+1].append(max_time)
        duration_dict[j+1].append(fitness_val)
        duration_dict[j+1].append(total_distance)
        duration_dict[j+1].append(total_time)
        duration_dict[j+1].append(depotb_vel)
        duration_dict[j+1].append(ugv_vel)
        duration_dict[j+1].append(param_list[0])
        # duration_dict[j+1].append(param_list[5])
        evolution_with_obj_val1.append([param_list[0], param_list[1], param_list[2], fitness_val])
        # if j == 0:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization in parallel in GA_gen_1_"+str(n+1)+"_.csv", "w", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
        # else:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization in parallel in GA_gen_1_"+str(n+1)+"_.csv", "a", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
    return evolution_with_obj_val1


def ga_evaluation2(X):
    sorted_initial_population_copy = copy.deepcopy(X)
    duration_dict = {0: ['NW_stop', 'SE_stop', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps', 'start point position']}
    for j in range(len(sorted_initial_population_copy)):
        param_list = sorted_initial_population_copy.pop(0)
        # print(param_list[5])
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        if penalty > 0:
            fitness_val += penalty
        if fitness_val == 0 or total_distance == 0 or total_time == 0:
            fitness_val = 50_000_000
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
        #     fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        #     if penalty > 0:
        #         fitness_val += penalty
        #     if fitness_val == 0 or total_distance == 0 or total_time == 0:
        #         fitness_val = 50_000_000
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        duration_dict[j+1].append(max_time)
        duration_dict[j+1].append(fitness_val)
        duration_dict[j+1].append(total_distance)
        duration_dict[j+1].append(total_time)
        duration_dict[j+1].append(depotb_vel)
        duration_dict[j+1].append(ugv_vel)
        duration_dict[j+1].append(param_list[0])
        # duration_dict[j+1].append(param_list[5])
        evolution_with_obj_val2.append([param_list[0], param_list[1], param_list[2], fitness_val])
        # if j == 0:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization in parallel in GA_gen_1_"+str(n+1)+"_.csv", "w", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
        # else:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization in parallel in GA_gen_1_"+str(n+1)+"_.csv", "a", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
    return evolution_with_obj_val2


def ga_evaluation_3(X, Y):
    evolution_gen = operator_selection(X, Y)
    evolution_with_obj_val1 = []
    evolution_copy = copy.deepcopy(evolution_gen)
    duration_dict = {0: ['NW_stop', 'SE_stop', 'T_DepotB', 'UGV_t', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps']}
    for j in range(len(evolution_gen)):
        param_list = evolution_copy.pop(0)
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        if penalty > 0:
            fitness_val += penalty
        if fitness_val == 0 or total_distance == 0 or total_time == 0:
            fitness_val = 50_000_000
        # if fitness_val < 1000000:
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
        #     fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        #     if penalty > 0:
        #         fitness_val += penalty
        #     if fitness_val == 0 or total_distance == 0 or total_time == 0:
        #         fitness_val = 50_000_000
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        duration_dict[j+1].append(max_time)
        duration_dict[j+1].append(fitness_val)
        duration_dict[j+1].append(total_distance)
        duration_dict[j+1].append(total_time)
        duration_dict[j+1].append(depotb_vel)
        duration_dict[j+1].append(ugv_vel)
        duration_dict[j+1].append(param_list[0])
        evolution_with_obj_val1.append([param_list[0], param_list[1], param_list[2], fitness_val])
        # if j == 0:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization_10gen in parallel in GA_gen_2_"+str(n+1)+"_.csv", "w", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
        # else:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization_10gen in parallel GA_gen_2_"+str(n+1)+"_.csv", "a", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
    return evolution_with_obj_val1


def ga_evaluation_4(X, Y):
    evolution_gen = operator_selection(X, Y)
    evolution_with_obj_val2 = []
    evolution_copy = copy.deepcopy(evolution_gen)
    duration_dict = {0: ['NW_stop', 'SE_stop', 'T_DepotB', 'UGV_t', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps']}
    for j in range(len(evolution_gen)):
        param_list = evolution_copy.pop(0)
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        if penalty > 0:
            fitness_val += penalty
        if fitness_val == 0 or total_distance == 0 or total_time == 0:
            fitness_val = 50_000_000
        # if fitness_val < 1000000:
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
        #     fitness_val, max_time, total_distance, total_time, depotb_vel, ugv_vel, penalty, route_dict, route, stop1, stop2, tw_dict = UIC_UMD_collab_innerloop_opt.main(param_list[0], param_list[1], param_list[2])
        #     if penalty > 0:
        #         fitness_val += penalty
        #     if fitness_val == 0 or total_distance == 0 or total_time == 0:
        #         fitness_val = 50_000_000
        duration_dict[j+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2]]
        duration_dict[j+1].append(max_time)
        duration_dict[j+1].append(fitness_val)
        duration_dict[j+1].append(total_distance)
        duration_dict[j+1].append(total_time)
        duration_dict[j+1].append(depotb_vel)
        duration_dict[j+1].append(ugv_vel)
        duration_dict[j+1].append(param_list[0])
        evolution_with_obj_val2.append([param_list[0], param_list[1], param_list[2], fitness_val])
        # if j == 0:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization_10gen in parallel in GA_gen_2_"+str(n+1)+"_.csv", "w", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
        # else:
        #     csv_file = open("GA generation dataset/ICUAS '22 time optimization_10gen in parallel GA_gen_2_"+str(n+1)+"_.csv", "a", newline='')
        #     writer = csv.writer(csv_file)
        #     writer.writerow(duration_dict[j+1])
    return evolution_with_obj_val2


def wrapper1(func, arg, queue):
    queue.put(func(arg))


def wrapper2(func, arg1, arg2, queue):
    queue.put(func(arg1, arg2))


t0 = time.time()
# Hyperparameters
np.random.seed(123)
n_samples = 30
number_of_generations = 10
space = Space([(2, 58), (120//60, 3000//60), (120//60, 3000//60)])
lhs = Lhs(lhs_type="classic", criterion=None)
x = lhs.generate(space.dimensions, n_samples)
print(x)
# print(len(x))

# Consolidating possible UGV stop location pairs into a hash table (dictionary). Each key of the dictionary represents the corresponding UGV stop location pairs.
# The first parameter in the 'space' is the key of the 'ugv_stops_dict' dictionary.
ugv_stops_dict = {}
ugv_Stops = [[(3.1, 7.24), (7.45, 3.86)], [(2.77, 7.57), (7.7, 3.61)], [(2.43, 7.91), (8.04, 3.28)], [(2.61, 8.34), (8.39, 2.98)], [(3.01, 8.87), (8.75, 2.73)], [(3.29, 9.23), (9.13, 2.5)], [(3.42, 9.61), (9.53, 2.29)], [(3.1, 7.24), (7.7, 3.61)], [(3.1, 7.24), (8.04, 3.28)], [(3.1, 7.24), (8.39, 2.98)], [(3.1, 7.24), (8.75, 2.73)], [(3.1, 7.24), (9.13, 2.5)], [(3.1, 7.24), (9.53, 2.29)],
             [(2.77, 7.57), (7.45, 3.86)], [(2.77, 7.57), (8.04, 3.28)], [(2.77, 7.57), (8.39, 2.98)], [(2.77, 7.57), (8.75, 2.73)], [(2.77, 7.57), (9.13, 2.5)], [(2.77, 7.57), (9.53, 2.29)], [(2.43, 7.91), (7.45, 3.86)], [(2.43, 7.91), (7.7, 3.61)], [(2.43, 7.91), (8.39, 2.98)], [(2.43, 7.91), (8.75, 2.73)], [(2.43, 7.91), (9.13, 2.5)], [(2.43, 7.91), (9.53, 2.29)], [(2.61, 8.34), (7.45, 3.86)],
             [(2.61, 8.34), (7.7, 3.61)], [(2.61, 8.34), (8.04, 3.28)], [(2.61, 8.34), (8.75, 2.73)], [(2.61, 8.34), (9.13, 2.5)], [(2.61, 8.34), (9.53, 2.29)], [(3.01, 8.87), (7.45, 3.86)], [(3.01, 8.87), (7.7, 3.61)], [(3.01, 8.87), (8.04, 3.28)], [(3.01, 8.87), (8.39, 2.98)], [(3.01, 8.87), (9.13, 2.5)], [(3.01, 8.87), (9.53, 2.29)],
             [(3.29, 9.23), (7.45, 3.86)], [(3.29, 9.23), (7.7, 3.61)], [(3.29, 9.23), (8.04, 3.28)], [(3.29, 9.23), (8.39, 2.98)], [(3.29, 9.23), (8.75, 2.73)], [(3.29, 9.23), (9.53, 2.29)], [(3.42, 9.61), (7.45, 3.86)], [(3.42, 9.61), (7.7, 3.61)], [(3.42, 9.61), (8.04, 3.28)], [(3.42, 9.61), (8.39, 2.98)], [(3.42, 9.61), (8.75, 2.73)], [(3.42, 9.61), (9.13, 2.5)], [(3.42, 9.61), (10.54, 0.9)],
             [(3.74, 10.45), (10.54, 0.9)], [(3.01, 8.87), (10.54, 0.9)], [(2.43, 7.91), (10.54, 0.9)], [(2.61, 8.34), (10.54, 0.9)], [(2.77, 7.57), (10.54, 0.9)], [(3.54, 10.04), (10.54, 0.9)], [(3.1, 7.24), (10.54, 0.9)]]

# rev_ugv_stops_lst = copy.deepcopy(ugv_Stops)
#
# for i in range(len(rev_ugv_stops_lst)):
#     rev_ugv_stops_lst[i].reverse()
#
# for l in range(len(rev_ugv_stops_lst)):
#     ugv_Stops.append(rev_ugv_stops_lst[l])

# print(ugv_Stops)

for i in range(len(ugv_Stops)):
    ugv_stops_dict[i+2] = ugv_Stops[i]

x_copy = x
q1 = Queue()
q2 = Queue()


'''Step 2: Selection, crossover, mutation'''
pop1 = []
pop2 = []
sorted_initial_population = x
for j in range(len(sorted_initial_population)):
    if j % 2 == 0:
        pop1.append(sorted_initial_population[j])
    else:
        pop2.append(sorted_initial_population[j])
gen_dict = {}
evolution_with_obj_val = []
evolution_with_obj_val1 = []
evolution_with_obj_val2 = []
evolution_gen = []
evol_1_split = []
evol_2_split = []
for n in range(number_of_generations):
    if n == 0:
        Thread(target=wrapper1, args=(ga_evaluation1, pop1, q1)).start()
        Thread(target=wrapper1, args=(ga_evaluation2, pop2, q2)).start()
        evo1 = q1.get()
        evo2 = q2.get()
        evolution_with_obj_val = evo1 + evo2
        print("The evolution population is: {}".format(len(evolution_with_obj_val)))
        evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[3]))
    else:
        evolution_gen = []
        for j in range(len(evolution_with_obj_val)):
            if j % 2 == 0:
                evol_1_split.append(evolution_with_obj_val[j])
            else:
                evol_2_split.append(evolution_with_obj_val[j])
        Thread(target=wrapper2, args=(ga_evaluation_3, pop1, evol_1_split, q1)).start()
        Thread(target=wrapper2, args=(ga_evaluation_4, pop2, evol_2_split, q2)).start()
        evo1 = q1.get()
        evo2 = q2.get()
        evolution_with_obj_val = evo1 + evo2
        evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[3]))
        print("The evolution population is: {}".format(len(evolution_with_obj_val)))
        evolution_with_obj_val_copy = copy.deepcopy(evolution_with_obj_val)
        '''After 'n' generations, print the optimal UGV route.'''
        print("The best fitness obtained in the current generation is: {}".format(evolution_with_obj_val[0][3]))
        for i in range(len(evolution_with_obj_val)):
            del evolution_with_obj_val_copy[i][3]
            evolution_gen.append(evolution_with_obj_val[i])
        # evolution_gen_array = np.array(evolution_gen)
        '''Step 3: Pass on to next generation population'''
        pop1 = []
        pop2 = []
        evol_1_split = []
        evol_2_split = []
        for e in range(len(evolution_gen)):
            if e % 2 == 0:
                pop1.append(evolution_gen[e])
            else:
                pop2.append(evolution_gen[e])
        for f in range(len(evolution_with_obj_val)):
            if f % 2 == 0:
                evol_1_split.append(evolution_with_obj_val[f])
            else:
                evol_2_split.append(evolution_with_obj_val[f])
t1 = time.time() - t0
print("Elapsed clock time = {}".format(t1))
