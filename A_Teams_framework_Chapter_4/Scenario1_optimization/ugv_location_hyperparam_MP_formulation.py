import copy
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
import random
from gurobipy import *


def get_loc_coordinates(idx):
    agent1_coord = loc_matrix[0][idx[0]]
    agent2_coord = loc_matrix[1][idx[1]]
    return agent1_coord, agent2_coord


def is_inside(circle_x, circle_y, rad, x, y):
    if (x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y) <= rad * rad:
        # print("Inside")
        return 1
    else:
        # print("Outside")
        return 0


# def f_matrix(combo_1, combo_2):
#     combo = [combo_1, combo_2]
#     radius = 23622/5280
#     feas_matrix = np.zeros((2, 120))
#     for j in range(120):
#         for i in range(2):
#             count = 0
#             border_list1 = []
#             sum_list = []
#             circle_x1 = combo[i][j][0][0]
#             circle_y1 = combo[i][j][0][1]
#             circle_x2 = combo[i][j][1][0]
#             circle_y2 = combo[i][j][1][1]
#             if i == 0:
#                 for k in range(len(data_list)):
#                     if 15 < k <= 31:
#                         x = data_list[k][0]
#                         y = data_list[k][1]
#                         cond1 = is_inside(circle_x1, circle_y1, radius, x, y)
#                         cond2 = is_inside(circle_x2, circle_y2, radius, x, y)
#                         border_list1.append([cond1, cond2])
#                         border = border_list1.pop(0)
#                         sum_list.append(sum(border))
#                 if sum_list.count(0) == 0:
#                     count += 1
#                 feas_matrix[i][j] = count
#             elif i == 1:
#                 for k in range(len(data_list)):
#                     if 31 < k <= 47:
#                         x = data_list[k][0]
#                         y = data_list[k][1]
#                         cond1 = is_inside(circle_x1, circle_y1, radius, x, y)
#                         cond2 = is_inside(circle_x2, circle_y2, radius, x, y)
#                         border_list1.append([cond1, cond2])
#                         border = border_list1.pop(0)
#                         sum_list.append(sum(border))
#                 if sum_list.count(0) == 0:
#                     count += 1
#                 feas_matrix[i][j] = count
#     '''Keep this section of code from lines 52-60 for future verification purposes'''
#     # for k in range(120):
#     #     for l in range(2):
#     #         count = 0
#     #         if (feas_matrix[0][k] + feas_matrix[1][k]) == 2:
#     #             count += 1
#     #         else:
#     #             count += 0
#     #         feas_matrix[0][k] += count
#     #         feas_matrix[1][k] += count
#     return feas_matrix

def f_matrix(combo_1, combo_2):
    combo = [combo_1, combo_2]
    radius = 23622/5280
    feas_matrix = np.zeros((2, 120))
    for j in range(120):
        for i in range(2):
            count = 0
            border_list1 = []
            sum_list = []
            circle_x1 = combo[0][j][i][0]
            circle_y1 = combo[0][j][i][1]
            circle_x2 = combo[1][j][i][0]
            circle_y2 = combo[1][j][i][1]
            for k in range(len(data_list)):
                if k <= 15:
                    continue
                x = data_list[k][0]
                y = data_list[k][1]
                cond1 = is_inside(circle_x1, circle_y1, radius, x, y)
                cond2 = is_inside(circle_x2, circle_y2, radius, x, y)
                border_list1.append([cond1, cond2])
                border = border_list1.pop(0)
                sum_list.append(sum(border))
            if sum_list.count(0) == 0:
                count += 1
            feas_matrix[i][j] = count
    '''Keep this section of code from lines 52-60 for future verification purposes'''
    for k in range(120):
        for l in range(2):
            count = 0
            if (feas_matrix[0][k] + feas_matrix[1][k]) == 2:
                count += 1
            else:
                count += 0
            feas_matrix[0][k] += count
            feas_matrix[1][k] += count
    return feas_matrix


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
# agent_2.reverse()
cost_matrix = np.zeros((num_branches, len(agent_1)))
# loc_matrix = [[] * len(agent_1)] * 2
cost_matrix = cost_matrix.astype(int)
fs_matrix = f_matrix(agent_1, agent_2)
zero_matrix = np.zeros((118, 120))
fs_matrix = np.vstack([fs_matrix, zero_matrix])
print(fs_matrix)
col_ind = []

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
cost_matrix = np.vstack([cost_matrix, zero_matrix])
loc_matrix = [agent_1_id_list, agent_2_id_list]


# Defining the Model
m = Model('Assignment')

# Defining the variable
x = {}
for i in range(len(cost_matrix)):
    for j in range(len(cost_matrix)):
        x[i, j] = m.addVar(vtype=GRB.BINARY, name="x%d,%d" % (i, j))

# constraints
for i in range(len(cost_matrix)):
    m.addConstr(quicksum(x[i, j] for j in range(len(cost_matrix))) == 1)
m.update()

for j in range(len(cost_matrix)):
    m.addConstr(quicksum(x[i, j] for i in range(len(cost_matrix))) == 1)
m.update()


m.addConstr(quicksum(fs_matrix[i][j]*x[i, j] for i in range(len(cost_matrix)) for j in range(len(cost_matrix))) == 4)
m.update()

# for i in range(len(cost_matrix)):
#     m.addConstr(quicksum(ch_matrix[i]*x[i, j] for j in range(len(cost_matrix))) == 1)

m.write("myLP.lp")

# Objective function
m.setObjective(quicksum(cost_matrix[i][j]*x[i, j] for i in range(len(cost_matrix)) for j in range(len(cost_matrix))),
               GRB.MINIMIZE)

m.optimize()

sol_x = {}
X = np.empty([120, 120])
if m.status == GRB.OPTIMAL:
    sol_x = m.getAttr('x', x)
    for i in range(120):
        for j in range(120):
            X[i, j] = int(sol_x[i, j])
    print("Optimal assignment is: {}".format(m.objVal))
    print("Corresponding X values are: {}".format(X.astype('int32')))

for i in range(2):
    for j in range(len(X)):
        if X[i][j] == 1:
            col_ind.append(j)

loc_coordinates_agent1, loc_coordinates_agent2 = get_loc_coordinates(col_ind)
print(loc_coordinates_agent1, loc_coordinates_agent2)
