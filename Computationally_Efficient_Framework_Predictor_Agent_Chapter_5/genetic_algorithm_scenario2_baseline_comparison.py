"""This code is optimizing the parameters using Genetic algorithm
Functions applied in this Genetic Algorithm code are described as follows:

binary_to_decimal: This function decodes the chromosomes (binary encoding) into decimal values of the UGV parameters.
decimal_to_binary: This function encodes the UGV parameters into binary numbers.
operator_selection: This function performs the Selection, crossover and mutation operations of the Genetic Algorithm Each operation has specific function.
tournament_selection: This function performs the Tournament selection operation.
mate: This function performs the crossover and mutation operations.
bitstring_to_param_list: This function separates the chromosomes into appropriate UGV parameters that can be understood by the inner-loop optimization code."""

import numpy as np
import overall_PS_multiUGVstops_UGVUAVmoving_scenario2
import random
from skopt.space import Space
from skopt.sampler import Lhs
import pandas as pd
import copy
import minimum_set_cover_UGV_hyperparam_refined_scenario2
import time
from threading import Thread
from queue import Queue


code_switch = 0
random.seed(10)


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


def operator_selection(evolution_with_obj, stops_comb_len):  # This function takes current generation population obtained from evolution as input arguments
    """init_population_sorted: The population of the current generation is sorted and sent as the argument.

       evolution_with_obj: This argument is the same as the init_population_sorted but with an extra objective/fitness val column"""
    # Selection -> Tournament selection is to be done
    # pop_size = len(init_population_sorted)
    # init_pop_sorted_copy = copy.deepcopy(init_population_sorted)
    evolution_with_obj_copy = copy.deepcopy(evolution_with_obj)
    first_candidate_set = copy.deepcopy(evolution_with_obj)
    next_generation = []
    perm_list = permutation(evolution_with_obj)
    second_list_father = [perm_list[f] for f in range(len(perm_list)) if f % 2 == 0]
    second_list_mother = [perm_list[m] for m in range(len(perm_list)) if m % 2 == 1]
    print(f"Second list father: {second_list_father}")
    print(f"Second list mother: {second_list_mother}")
    # perform Ellitism: have some chromosomes from the previous generation. For example, 5% of population
    if len(evolution_with_obj) % 2 == 0:
        evolution_pop_size = len(evolution_with_obj)//2 + 1
        for i in range(evolution_pop_size):
            if i > len(evolution_with_obj):
                continue
            # print(i)
            if i < 2:
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
                objval_1 = father[4]
                objval_2 = mother[4]
                del father[-1]
                del mother[-1]
                print("Father after deletion: {}".format(father))
                print("Mother after deletion: {}".format(mother))
                # print(father)
                # print(mother)
                child1, child2 = mate(father, mother, stops_comb_len)
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
                print(f"Evolved solution 1: {child1_conv_decimal}")
                print(f"Evolved solution 2: {child2_conv_decimal}")
                father.insert(4, objval_1)
                mother.insert(4, objval_2)
    else:
        evolution_pop_size = len(evolution_with_obj)//2 + 1
        for i in range(evolution_pop_size):
            if i > len(evolution_with_obj):
                continue
            # print(i)
            if i < 1:
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
                objval_1 = father[4]
                objval_2 = mother[4]
                del father[-1]
                del mother[-1]
                print("Father after deletion: {}".format(father))
                print("Mother after deletion: {}".format(mother))
                # print(father)
                # print(mother)
                child1, child2 = mate(father, mother, stops_comb_len)
                child1_conv = bitstring_to_param_list(child1)
                # print(child1_conv)
                child2_conv = bitstring_to_param_list(child2)
                # print(child2_conv)
                child1_conv_decimal = binary_to_decimal(child1_conv)
                child2_conv_decimal = binary_to_decimal(child2_conv)
                next_generation.append(child1_conv_decimal)
                next_generation.append(child2_conv_decimal)
                print(f"Evolved solution 1: {child1_conv_decimal}")
                print(f"Evolved solution 2: {child2_conv_decimal}")
                father.insert(4, objval_1)
                mother.insert(4, objval_2)
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
    if candidate_1[4] <= candidate_2[4]:
        return candidate_1
    else:
        return candidate_2


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
    print(f"The permuted list is: {candidate_2_set} and its length is {len(candidate_2_set)}")
    return candidate_2_set


def mate(father, mother, stops_comb_len):  # This function performs crossover and mutation and takes two different UGV parameter lists as input arguments.
    bin_father, bin_father_str = decimal_to_binary(father)
    bin_mother, bin_mother_str = decimal_to_binary(mother)
    chromosomes_father = bin_father_str[0]+bin_father_str[1]+bin_father_str[2]+bin_father_str[3]
    chromosomes_mother = bin_mother_str[0]+bin_mother_str[1]+bin_mother_str[2]+bin_mother_str[3]
    print(chromosomes_father)
    print(chromosomes_mother)
    # 2-point Xover operation for normal Genetic algorithm operation
    k1 = int(round(random.uniform(1, (len(chromosomes_father)//2)), 0))
    k2 = int(round(random.uniform(k1, len(chromosomes_father)), 0))

    # Genetic algorithm operation post-sensitivity analysis
    # print(normalized_sensitivity)
    # max_prob = max(normalized_sensitivity)
    # print(f'Max probability val: {max_prob}')
    # max_prob_id = normalized_sensitivity.index(max_prob)
    # chromosome_gene_bf_xover_loc = max_prob_id*8
    # chromosome_gene_af_xover_loc = max_prob_id*8 + 8
    # removed_optimization_variable_father = chromosomes_father[chromosome_gene_bf_xover_loc:chromosome_gene_af_xover_loc]
    # removed_optimization_variable_mother = chromosomes_mother[chromosome_gene_bf_xover_loc:chromosome_gene_af_xover_loc]
    # chromosomes_father = chromosomes_father[:chromosome_gene_bf_xover_loc] + chromosomes_father[chromosome_gene_af_xover_loc:]
    # chromosomes_mother = chromosomes_mother[:chromosome_gene_bf_xover_loc] + chromosomes_mother[chromosome_gene_af_xover_loc:]
    # print(f'Crossover starting point: {chromosome_gene_bf_xover_loc}')
    # print(f'Crossover ending point: {chromosome_gene_af_xover_loc}')
    # k1 = int(round(random.uniform(1, (len(chromosomes_father)//2)), 0))

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

    # Crossover post-sensitivity analysis
    # for j in range(len(chromosomes_father)):
    #     if j < k1:
    #         offspring1 += chromosomes_father[j]
    #         offspring2 += chromosomes_mother[j]
    #     else:
    #         offspring1 += chromosomes_mother[j]
    #         offspring2 += chromosomes_father[j]
    print("Offspring1 before mutation: {}".format(offspring1))
    print("Offspring2 before mutation: {}".format(offspring2))
    # print(f"Father chromosome removed for Crossover: {removed_optimization_variable_father}")
    # print(f"Mother chromosome removed for Crossover: {removed_optimization_variable_mother}")

    # for i in range(len(offspring1)):
    #     if i < chromosome_gene_bf_xover_loc:
    #         continue
    #     elif chromosome_gene_bf_xover_loc <= i < chromosome_gene_af_xover_loc:
    #         offspring1 = offspring1[:chromosome_gene_bf_xover_loc] + removed_optimization_variable_father + offspring1[chromosome_gene_bf_xover_loc:]
    #         offspring2 = offspring2[:chromosome_gene_bf_xover_loc] + removed_optimization_variable_mother + offspring2[chromosome_gene_bf_xover_loc:]
    #         break
    #     else:
    #         continue
    # print("Offspring1 after adding the crossed-over chromosome: {}".format(offspring1))
    # print("Offspring2 after adding the crossed-over chromosome: {}".format(offspring2))

    # mutation operation - post-sensitivity analysis
    p_mutation = 0.01
    # mutation_id_strt = max_prob_id*8
    # mutation_id_end = (max_prob_id*8) + 8
    # for k in range(len(offspring1)):
    #     if k < 8:
    #         if random.random() < p_mutation:
    #             if offspring1[k] == '0':
    #                 offspring1[k].replace('0', '1')
    #             else:
    #                 offspring1[k].replace('1', '0')
    #         if offspring2[k] == '0':
    #             offspring2[k].replace('0', '1')
    #         else:
    #             offspring2[k].replace('1', '0')
    #     elif 8 <= k < 16:
    #         if random.random() < p_mutation2:
    #             if offspring1[k] == '0':
    #                 offspring1[k].replace('0', '1')
    #             else:
    #                 offspring1[k].replace('1', '0')
    #         if offspring2[k] == '0':
    #             offspring2[k].replace('0', '1')
    #         else:
    #             offspring2[k].replace('1', '0')
    #     elif 16 <= k < 24:
    #         if random.random() < p_mutation3:
    #             if offspring1[k] == '0':
    #                 offspring1[k].replace('0', '1')
    #             else:
    #                 offspring1[k].replace('1', '0')
    #         if offspring2[k] == '0':
    #             offspring2[k].replace('0', '1')
    #         else:
    #             offspring2[k].replace('1', '0')
    #     elif 24 <= k < 32:
    #         if random.random() < p_mutation4:
    #             if offspring1[k] == '0':
    #                 offspring1[k].replace('0', '1')
    #             else:
    #                 offspring1[k].replace('1', '0')
    #         if offspring2[k] == '0':
    #             offspring2[k].replace('0', '1')
    #         else:
    #             offspring2[k].replace('1', '0')
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
    stops_len_binary = int(bin(stops_comb_len).replace("0b", ""))
    if int(offspring1_list[0]) > 11010 or int(offspring1_list[0]) < 10:
        if int(offspring1_list[0]) > 11010:
            # str_binary = str(stops_len_binary)
            # rem_len = 8 - len(str_binary)
            # temp_string = ''
            # for i in range(rem_len):
            #     temp_string += '0'
            # temp_string += str_binary
            offspring1_list[0] = '00011010'
        if int(offspring1_list[0]) < 10:
            offspring1_list[0] = '00000010'
    if int(offspring1_list[1]) > 101 or int(offspring1_list[1]) < 10:
        if int(offspring1_list[1]) > 101:
            offspring1_list[1] = '00000101'
        if int(offspring1_list[1]) < 10:
            offspring1_list[1] = '00000100'
    if int(offspring1_list[2]) > 111 or int(offspring1_list[2]) < 10:
        if int(offspring1_list[2]) > 111:
            offspring1_list[2] = '00000111'
        if int(offspring1_list[2]) < 10:
            offspring1_list[2] = '00000010'
    if int(offspring1_list[3]) > 111 or int(offspring1_list[3]) < 10:
        if int(offspring1_list[3]) > 111:
            offspring1_list[3] = '00000111'
        if int(offspring1_list[3]) < 10:
            offspring1_list[3] = '00000010'
    offspring1 = offspring1_list[0]+offspring1_list[1]+offspring1_list[2]+offspring1_list[3]
    offspring2_list = bitstring_to_param_list(offspring2)
    if int(offspring2_list[0]) > 11010 or int(offspring2_list[0]) < 10:
        if int(offspring2_list[0]) > 11010:
            # str_binary = str(stops_len_binary)
            # rem_len = 8 - len(str_binary)
            # temp_string = ''
            # for i in range(rem_len):
            #     temp_string += '0'
            # str_binary += temp_string
            offspring2_list[0] = '00011010'
        if int(offspring2_list[0]) < 10:
            offspring2_list[0] = '00000010'
    if int(offspring2_list[1]) > 101 or int(offspring2_list[1]) < 10:
        if int(offspring2_list[1]) > 101:
            offspring2_list[1] = '00000101'
        if int(offspring2_list[1]) < 10:
            offspring2_list[1] = '00000010'
    if int(offspring2_list[2]) > 111 or int(offspring2_list[2]) < 10:
        if int(offspring2_list[2]) > 111:
            offspring2_list[2] = '00000111'
        if int(offspring2_list[2]) < 10:
            offspring2_list[2] = '00000010'
    if int(offspring2_list[3]) > 111 or int(offspring2_list[3]) < 10:
        if int(offspring2_list[3]) > 111:
            offspring2_list[3] = '00000111'
        if int(offspring2_list[3]) < 10:
            offspring2_list[3] = '00000010'
    offspring2 = offspring2_list[0]+offspring2_list[1]+offspring2_list[2]+offspring2_list[3]
    print("Offspring1 after mutation: {}".format(offspring1))
    print("Offspring2 after mutation: {}".format(offspring2))
    print(len(offspring1), len(offspring2))
    return offspring1, offspring2


def bitstring_to_param_list(bitstring):  # This function takes an entire chromosome as input argument and returns the appropriate UGV parameter list.
    parameter_list_conv = []
    param_1 = str()
    param_2 = str()
    param_3 = str()
    param_4 = str()
    # param_5 = str()
    # param_6 = str()
    for i in range(len(bitstring)):
        if i < 8:
            param_1 += bitstring[i]
        elif 8 <= i < 16:
            param_2 += bitstring[i]
        elif 16 <= i < 24:
            param_3 += bitstring[i]
        elif 24 <= i < 32:
            param_4 += bitstring[i]
        # elif 32 <= i < 40:
        #     param_5 += bitstring[i]
        # elif 40 <= i < 48:
        #     param_6 += bitstring[i]
    parameter_list_conv.append(param_1)
    parameter_list_conv.append(param_2)
    parameter_list_conv.append(param_3)
    parameter_list_conv.append(param_4)
    return parameter_list_conv


'''param_list_order = [ugv_stop_loc, ugv_depotb_to_ugvstop_velocity, ugv_velocity_between_two_ugvstops, NW_wait_time, SE_wait_time, period_starting_location]'''


def ga_evaluation1(X):
    sorted_initial_population_copy = copy.deepcopy(X)
    for j in range(len(sorted_initial_population_copy)):
        print("Evaluated using ga_evaluation1")
        param_list = sorted_initial_population_copy.pop(0)
        global_f, global_t, global_tt, global_depotvel, global_ugvvel, global_track_node_age, global_route_dict, global_route_dict2, global_route_dict3, global_sp_feas_list, global_total_cost = overall_PS_multiUGVstops_UGVUAVmoving_scenario2.persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
        evolution_with_obj_val1.append([param_list[0], param_list[1], param_list[2], param_list[3],  global_total_cost])
    return evolution_with_obj_val1


def ga_evaluation2(X):
    sorted_initial_population_copy = copy.deepcopy(X)
    for j in range(len(sorted_initial_population_copy)):
        print("Evaluated using ga_evaluation2")
        param_list = sorted_initial_population_copy.pop(0)
        global_f, global_t, global_tt, global_depotvel, global_ugvvel, global_track_node_age, global_route_dict, global_route_dict2, global_route_dict3, global_sp_feas_list, global_total_cost = overall_PS_multiUGVstops_UGVUAVmoving_scenario2.persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
        evolution_with_obj_val2.append([param_list[0], param_list[1], param_list[2], param_list[3], global_total_cost])
    return evolution_with_obj_val2


def ga_evaluation_3(X):
    print(f"Evolution gen length of ga_evaluation_3 is: {len(X)}")
    evolution_gen = operator_selection(X, stops_comb_len)
    evolution_with_obj_val1 = []
    evolution_copy = copy.deepcopy(evolution_gen)
    for j in range(len(evolution_gen)):
        print("Evaluated using ga_evaluation3")
        param_list = evolution_copy.pop(0)
        global_f, global_t, global_tt, global_depotvel, global_ugvvel, global_track_node_age, global_route_dict, global_route_dict2, global_route_dict3, global_sp_feas_list, global_total_cost = overall_PS_multiUGVstops_UGVUAVmoving_scenario2.persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
        evolution_with_obj_val1.append([param_list[0], param_list[1], param_list[2], param_list[3], global_total_cost])
    return evolution_with_obj_val1


def ga_evaluation_4(X):
    print(f"Evolution gen length of ga_evaluation_4 is: {len(X)}")
    evolution_gen = operator_selection(X, stops_comb_len)
    evolution_with_obj_val2 = []
    evolution_copy = copy.deepcopy(evolution_gen)
    for j in range(len(evolution_gen)):
        print("Evaluated using ga_evaluation4")
        param_list = evolution_copy.pop(0)
        global_f, global_t, global_tt, global_depotvel, global_ugvvel, global_track_node_age, global_route_dict, global_route_dict2, global_route_dict3, global_sp_feas_list, global_total_cost = overall_PS_multiUGVstops_UGVUAVmoving_scenario2.persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
        evolution_with_obj_val2.append([param_list[0], param_list[1], param_list[2], param_list[3], global_total_cost])
    return evolution_with_obj_val2


def wrapper1(func, arg, queue):
    queue.put(func(arg))


def wrapper2(func, arg1, queue):
    queue.put(func(arg1))


'''Step 1: Initial population'''  # Generate using Latin Hypercube Sampling (LHS)
ugv_max_capacity = 25_010_000  # in J
ugv_velocity = 15  # in ft/s
uav_speed = 33  # in ft/s
strt_pt = [(0.6, 8.07)]
# Hyperparameters

t0 = time.time()
np.random.seed(111)
print("---------------------Random seed number = 111-------------------")
n_samples = 40
num_uavs = 1
"""Ensure that the changes that you make here is properly reflected in the 'genetic_algorithm' module"""
# Consolidating possible UGV stop location pairs into a hash table (dictionary)
ordered_pts_from_depot = pd.read_csv('Scenario 6 ordered from depot.csv')
ordered_pts_rest_df = pd.read_csv('Scenario 6 ordered rest.csv')
ordered_pts_from_depot = ordered_pts_from_depot.values.tolist()
ordered_pts_rest = ordered_pts_rest_df.values.tolist()
ordered_pts_from_depot = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot]
ordered_pts_rest = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_rest]
ordered_pts_from_depot_second = pd.read_csv('Scenario 6 ordered from depot second.csv')
ordered_pts_from_depot_second = ordered_pts_from_depot_second.values.tolist()
ordered_pts_from_depot_second = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot_second]

reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))
ugv_stops_dict = {}
ugv_Stops = minimum_set_cover_UGV_hyperparam_refined_scenario2.main(ordered_pts_rest_df, 'Case_Study_scenario6.csv')


# Updated UGV end pairs for replanning
# ugv_stops_dict = {}
# ugv_Stops = [[(3.43, 6.9), (7.45, 3.86)], [(3.43, 6.9), (7.62, 3.69)], [(3.43, 6.9), (7.7, 3.61)], [(3.43, 6.9), (8.04, 3.28)]]

for i in range(len(ugv_Stops)):
    ugv_stops_dict[i+2] = ugv_Stops[i]

stops_comb_len = len(ugv_Stops)
print(f"Total number of UGV end point combinations: {stops_comb_len}")
# TODO: Tune the genetic algorithm chromosomes accordingly when the hyperparam set is tuned.
space = Space([(2, stops_comb_len+1), (2, 5), (2, 7), (2, 7)])
space_list = [(2, stops_comb_len+1), (2, 5), (2, 7), (2, 7)]
lhs = Lhs(lhs_type="classic", criterion='correlation')
x = lhs.generate(space.dimensions, n_samples)
# for l in range(len(x)):
#     x[l].append(random.choices([2, 6, 11], weights=[5, 3, 1], k=1)[0])
# for l in range(len(x)):
#     val = 2
#     x[l][3] = val
print(x)
number_of_generations = 20

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
population_dict = {}
q1 = Queue()
q2 = Queue()
for n in range(number_of_generations):
    # print("---------------------Random seed number = 111-------------------")
    print("Generation number is: {}".format(n+1))
    t1_1 = time.time() - t0
    print("Elapsed clock time = {}".format(t1_1))
    if n == 0:
        # evo1 = ga_evaluation1(pop1)
        # evo2 = ga_evaluation2(pop2)
        Thread(target=wrapper1, args=(ga_evaluation1, pop1, q1)).start()
        Thread(target=wrapper1, args=(ga_evaluation2, pop2, q2)).start()
        evo1 = q1.get()
        evo2 = q2.get()
        evolution_with_obj_val = evo1 + evo2
        evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[4]))
        print("The evolution population is: {}".format(len(evolution_with_obj_val)))
        print("Evolved solutions are: {}".format(evolution_with_obj_val))
        gen_dict[n] = evolution_gen
    else:
        evolution_gen = []
        if n == 1:
            for j in range(len(evolution_with_obj_val)):
                if j % 2 == 0:
                    evol_1_split.append(evolution_with_obj_val[j])
                else:
                    evol_2_split.append(evolution_with_obj_val[j])
        Thread(target=wrapper2, args=(ga_evaluation_3, evol_1_split, q1)).start()
        Thread(target=wrapper2, args=(ga_evaluation_4, evol_2_split, q2)).start()
        evo1 = q1.get()
        evo2 = q2.get()
        # evo1 = ga_evaluation_3(pop1, evol_1_split)
        # evo2 = ga_evaluation_4(pop2, evol_2_split)
        evolution_with_obj_val = evo1 + evo2
        evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[4]))
        evolution_with_obj_val_copy = copy.deepcopy(evolution_with_obj_val)
        print("The evolution population is: {}".format(len(evolution_with_obj_val)))
        print("Evolved solutions are: {}".format(evolution_with_obj_val))
        '''After 'n' generations, print the optimal UGV route.'''
        print("The best fitness obtained in the current generation with parameters {}, {}, {} and {} is: {}".format(evolution_with_obj_val[0][0], evolution_with_obj_val[0][1], evolution_with_obj_val[0][2], evolution_with_obj_val[0][3], evolution_with_obj_val[0][4]))
        for i in range(len(evolution_with_obj_val)):
            del evolution_with_obj_val_copy[i][4]
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
        if n >= 1:
            for f in range(len(evolution_with_obj_val)):
                if f % 2 == 0:
                    evol_1_split.append(evolution_with_obj_val[f])
                else:
                    evol_2_split.append(evolution_with_obj_val[f])
        gen_dict[n] = evolution_gen
        if gen_dict[n-1] == gen_dict[n]:
            break
t1 = time.time() - t0
print("Elapsed clock time = {}".format(t1))
