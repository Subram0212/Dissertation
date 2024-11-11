"""This code is optimizing the parameters using Genetic algorithm
Functions applied in this Genetic Algorithm code are described as follows:

binary_to_decimal: This function decodes the chromosomes (binary encoding) into decimal values of the UGV parameters.
decimal_to_binary: This function encodes the UGV parameters into binary numbers.
operator_selection: This function performs the Selection, crossover and mutation operations of the Genetic Algorithm Each operation has specific function.
tournament_selection: This function performs the Tournament selection operation.
mate: This function performs the crossover and mutation operations.
bitstring_to_param_list: This function separates the chromosomes into appropriate UGV parameters that can be understood by the inner-loop optimization code."""

import numpy as np
import scenario2_innerloop_opt
import random
from skopt.space import Space
from skopt.sampler import Lhs
import csv
import pandas as pd
import copy
import scenario_2_data_processing
from itertools import combinations
import scenario2_innerloop_opt_updated_solution



code_switch = 0


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


def operator_selection(evolution_with_obj):  # This function takes current generation population obtained from evolution as input arguments
    """init_population_sorted: The population of the current generation is sorted and sent as the argument.

       evolution_with_obj: This argument is the same as the init_population_sorted but with an extra objective/fitness val column"""
    # Selection -> Tournament selection is to be done
    # pop_size = len(init_population_sorted)
    # init_pop_sorted_copy = copy.deepcopy(init_population_sorted)
    evolution_with_obj_copy = copy.deepcopy(evolution_with_obj)
    first_candidate_set = copy.deepcopy(evolution_with_obj)
    next_generation = []
    perm_list = permutation(evolution_with_obj)
    second_list_father = [f for f in perm_list if perm_list.index(f) % 2 == 0]
    second_list_mother = [m for m in perm_list if perm_list.index(m) % 2 == 1]
    # perform Ellitism: have some chromosomes from the previous generation. For example, 5% of population
    for i in range(len(evolution_with_obj)//2 + 2):
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
            objval_1 = father[4]
            objval_2 = mother[4]
            del father[-1]
            del mother[-1]
            print("Father after deletion: {}".format(father))
            print("Mother after deletion: {}".format(mother))
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
    return candidate_2_set


def mate(father, mother):  # This function performs crossover and mutation and takes two different UGV parameter lists as input arguments.
    bin_father, bin_father_str = decimal_to_binary(father)
    bin_mother, bin_mother_str = decimal_to_binary(mother)
    chromosomes_father = bin_father_str[0]+bin_father_str[1]+bin_father_str[2]+bin_father_str[3]
    chromosomes_mother = bin_mother_str[0]+bin_mother_str[1]+bin_mother_str[2]+bin_mother_str[3]
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
    if int(offspring1_list[0]) > 1001 or int(offspring1_list[0]) < 10:
        if int(offspring1_list[0]) > 1001:
            offspring1_list[0] = '00001001'
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
    if int(offspring1_list[3]) > 101 or int(offspring1_list[3]) < 10:
        if int(offspring1_list[3]) > 101:
            offspring1_list[3] = '00000101'
        if int(offspring1_list[3]) < 10:
            offspring1_list[3] = '00000010'
    offspring1 = offspring1_list[0]+offspring1_list[1]+offspring1_list[2]+offspring1_list[3]
    offspring2_list = bitstring_to_param_list(offspring2)
    if int(offspring2_list[0]) > 1001 or int(offspring2_list[0]) < 10:
        if int(offspring2_list[0]) > 1001:
            offspring2_list[0] = '00001001'
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
    if int(offspring2_list[3]) > 101 or int(offspring2_list[3]) < 10:
        if int(offspring2_list[3]) > 101:
            offspring2_list[3] = '00000101'
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
    # parameter_list_conv.append(param_5)
    # parameter_list_conv.append(param_6)
    return parameter_list_conv
