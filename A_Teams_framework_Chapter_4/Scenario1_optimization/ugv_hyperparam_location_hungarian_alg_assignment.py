import copy
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment
from itertools import combinations
import random


def get_loc_coordinates(idx):
    agent1_coord = loc_matrix[0][idx[0]]
    agent2_coord = loc_matrix[1][idx[1]]
    return agent1_coord, agent2_coord


def is_inside(circle_x, circle_y, rad, x, y):
    if (x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y) <= rad * rad:
        print("Inside")
        cond = 'Inside'
        return True, cond
    else:
        print("Outside")
        cond = 'Outside'
        return False, cond


def loc_feasibility(locations, branch):
    binary_value = 1
    radius = 23622/5280
    depot_b_x = 0
    depot_b_y = 0
    circle_x1 = locations[0][0]
    circle_y1 = locations[0][1]
    circle_x2 = locations[1][0]
    circle_y2 = locations[1][1]
    for i in range(len(data_list)):
        border_list = []
        # if 15 < i <= 31:
        x = data_list[i][0]
        y = data_list[i][1]
        cond, border = is_inside(depot_b_x, depot_b_y, radius, x, y)
        # if not cond:
        #     binary_value = 0
        border_list.append(border)
        cond, border = is_inside(circle_x1, circle_y1, radius, x, y)
        # if not cond:
        #     binary_value = 0
        border_list.append(border)
        cond, border = is_inside(circle_x2, circle_y2, radius, x, y)
        # if not cond:
        #     binary_value = 0
        border_list.append(border)
        new_dict = {(x, y): border_list}
        feasibility.update(new_dict)
    for key, val in feasibility.items():
        if feasibility[key] == ['Outside', 'Outside', 'Outside']:
            binary_value = 0
    return binary_value, circle_x1, circle_y1


data = pd.read_csv('Scenario dataset/Scenario 1 data points.csv')
data_list = data.values.tolist()
print(data)
tasks_1 = []
tasks_2 = []
feasibility = {}
num_branches = 2
num_stops_each_branch = 2
bin_value = 0
discard_list = []
for i in range(len(data)):
    if 15 < i <= 31:
        tasks_1.append([data['x (miles)'][i], data['y (miles)'][i]])
    if 31 < i <= 47:
        tasks_2.append([data['x (miles)'][i], data['y (miles)'][i]])
agent_1 = list(combinations(tasks_1, num_branches))
agent_2 = list(combinations(tasks_2, num_branches))
# random.shuffle(agent_1)
# random.shuffle(agent_2)
# agent_1.reverse()
cost_matrix = np.zeros((num_branches, len(agent_1)))
# loc_matrix = [[] * len(agent_1)] * 2
cost_matrix = cost_matrix.astype(int)

id = 0
agent_1_id_list = []
agent_2_id_list = []
for j in agent_1:
    loc_1_id = data_list.index(j[0])
    loc_2_id = data_list.index(j[1])
    agent_1_id_list.append([data_list[loc_1_id], data_list[loc_2_id]])
    diff = abs(loc_1_id - loc_2_id) + 1
    cost_matrix[0][id] = diff
    # loc_matrix[0].append([data_list[loc_1_id], data_list[loc_2_id]])
    id += 1

id_2 = 0
for k in agent_2:
    loc_1_id = data_list.index(k[0])
    loc_2_id = data_list.index(k[1])
    agent_2_id_list.append([data_list[loc_1_id], data_list[loc_2_id]])
    diff = abs(loc_1_id - loc_2_id) + 1
    cost_matrix[1][id_2] = diff
    # loc_matrix[1].append([data_list[loc_1_id], data_list[loc_2_id]])
    id_2 += 1

cost_matrix_list = cost_matrix.tolist()
cost_matrix_one_branch = cost_matrix_list[0]
cost_matrix_another_branch = cost_matrix_list[1]

loc_matrix_one_branch = copy.deepcopy(agent_1_id_list)
loc_matrix_another_branch = copy.deepcopy(agent_2_id_list)

loc_matrix = [agent_1_id_list, agent_2_id_list]

while bin_value == 0 and len(cost_matrix[0]) > 2:
    print(cost_matrix)
    print(loc_matrix)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    print(col_ind)
    print(cost_matrix[row_ind, col_ind].sum())
    loc_coordinates_agent1, loc_coordinates_agent2 = get_loc_coordinates(col_ind)
    print(loc_coordinates_agent1, loc_coordinates_agent2)

    for i in range(num_stops_each_branch):
        if i == 0:
            locations = [loc_coordinates_agent1[i], loc_coordinates_agent2[i]]
            bin_value, x, y = loc_feasibility(locations, i)
            if bin_value == 0:
                cost_matrix_one_branch.pop(col_ind[i])
                loc_matrix_one_branch.pop(col_ind[i])
        elif i == 1:
            locations = [loc_coordinates_agent1[i], loc_coordinates_agent2[i]]
            bin_value, x, y = loc_feasibility(locations, i)
            if bin_value == 0:
                cost_matrix_another_branch.pop(col_ind[i])
                loc_matrix_another_branch.pop(col_ind[i])
    if len(cost_matrix_one_branch) > len(cost_matrix_another_branch):
        cost_matrix_one_branch.pop(col_ind[0])
        loc_matrix_one_branch.pop(col_ind[0])
        bin_value = 0
    elif len(cost_matrix_one_branch) < len(cost_matrix_another_branch):
        cost_matrix_another_branch.pop(col_ind[1])
        loc_matrix_another_branch.pop(col_ind[1])
        bin_value = 0
    cost_matrix_list = [cost_matrix_one_branch, cost_matrix_another_branch]
    loc_matrix = [loc_matrix_one_branch, loc_matrix_another_branch]
    cost_matrix = np.array(cost_matrix_list)
    print("cost matrix branches length: ", len(cost_matrix_one_branch), len(cost_matrix_another_branch))
