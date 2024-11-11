import numpy as np
import pandas as pd
import genetic_algorithm_moving_UGVUAV as ga
from skopt.space import Space
from skopt.sampler import Lhs
import copy
from scipy.optimize import minimize
from threading import Thread
from queue import Queue
import time
import matplotlib.pyplot as plt
import overall_PS_multiUGVstops_UGVUAVmoving_scenario2 as PS
import minimum_set_cover_UGV_hyperparam_refined_scenario2 as minimum_set_cover_UGV_hyperparam_refined
import ugv_distance_calc_auto_scenario2 as ugv_distance_calc_auto
# from sensitivity_analysis_morrismethod_function import MorrisMethod
from sklearn.cluster import KMeans
from sklearn.mixture import BayesianGaussianMixture
from sklearn.ensemble import StackingClassifier
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.svm import SVC
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score
# import gaussian_process_regressor as gp
from sklearn.base import BaseEstimator, ClassifierMixin
# import GPy
import scipy.stats as sp
import yaml
import random


def perch_and_move_agents(location, sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest):
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
                    temp = ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(ordered_pts_from_depot[j], ordered_pts_from_depot[j+1], ordered_pts_from_depot) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(ordered_pts_from_depot[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [ordered_pts_from_depot[-2]]
    elif sp2 == 1:
        for i in ordered_pts_rest:
            if i != location[0]:
                cnt += 1
                continue
            else:
                for j in range(cnt, len(ordered_pts_rest)):
                    if j == len(ordered_pts_rest)-1:
                        break
                    temp = ugv_distance_calc_auto.distance_btw_ugvstops(ordered_pts_rest[j], ordered_pts_rest[j+1], ordered_pts_rest) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(ordered_pts_rest[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [ordered_pts_rest[-1]]
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
                    temp = ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ordered_pts_rest[j], reversed_ordered_pts_rest[j+1], ordered_pts_rest) // ugv_velocity
                    cumul_dist += temp
                    if cumul_dist >= recharge_constant_time:
                        uav_act_depart_location.append(reversed_ordered_pts_rest[j])
                        break
                if len(uav_act_depart_location) == 0:
                    uav_act_depart_location = [reversed_ordered_pts_rest[-1]]
    return uav_act_depart_location


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


def unique_evolution(X_list):
    unique_list = []
    unique_list_stops = []
    ugv_stops = []
    for i in X_list:
        ugv_stops.append(i[0])
    for j in ugv_stops:
        if j not in unique_list_stops:
            unique_list_stops.append(j)
    for i in range(len(X_list)):
        if len(unique_list_stops) == 0:
            break
        if unique_list_stops[0] == X_list[i][0]:
            unique_list.append(X_list[i])
            unique_list_stops.pop(0)
    return unique_list


# class CustomGaussianProcessClassifier(BaseEstimator, ClassifierMixin):
#     def __init__(self, kernel=None, alpha=1e-10):
#         self.kernel = kernel
#         self.alpha = alpha
#         self.gp = None
#
#     def fit(self, X, y):
#         self.gp = GPy.models.GPClassification(X, y.reshape(-1, 1), kernel=self.kernel)
#         self.gp.optimize_restarts(num_restarts=1)
#
#     def predict(self, X):
#         predictions, var = self.gp.predict(X)
#         predictions = (predictions >= 0.6).astype(int)
#         return predictions


def param_to_distance_mapping(population, ordered_pts_from_depot, ordered_pts_rest):
    cnt = 0
    if isinstance(population, dict):
        for key, val in population_dict.items():
            if val > 500_000_000:
                continue
            loc_pair = ugv_stops_dict[key[0]]
            count_1 = None
            count_2 = None
            data_flag = 0
            for i, list_val in enumerate(ordered_pts_from_depot):
                if ordered_pts_from_depot[i] == (round(strt_pt[0][0], 2), round(strt_pt[0][1], 2)):
                    count_1 = i
                if ordered_pts_from_depot[i] == (round(loc_pair[0][0], 2), round(loc_pair[0][1], 2)):
                    count_2 = i
            if count_1 or count_2 is None:
                ordered_pts_from_depot = ordered_pts_from_depot_second
                ordered_pts_rest = list(reversed(ordered_pts_rest))
                data_flag = 1
            if data_flag == 1:
                ordered_pts_from_depot = ordered_pts_from_depot_second
                ordered_pts_rest = list(reversed(ordered_pts_rest))
            ugv_ends_dist = ugv_distance_calc_auto.distance_btw_ugvstops(loc_pair[0], loc_pair[1], ordered_pts_rest)
            itm_stp_1_dist = 0
            itm_stp_2_dist = 0
            itm_stp_3_dist = 0
            sp1 = 0
            sp2 = 0
            sp3 = 0
            for idx, value in enumerate(ordered_pts_from_depot):
                if idx == key[1]:
                    sp1 += 1
                    itm_stp_1 = value
                    itm_stp_1_dist += ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(strt_pt[0], itm_stp_1, ordered_pts_from_depot)
                    next_loc = perch_and_move_agents([itm_stp_1], sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest)
                    end_loc_id = ordered_pts_from_depot.index(next_loc[0]) + key[1]
                    if end_loc_id >= len(ordered_pts_from_depot):
                        end_loc = ordered_pts_from_depot[-1]
                    else:
                        end_loc = ordered_pts_from_depot[end_loc_id]
                    itm_stp_1_dist += ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(next_loc[0], end_loc, ordered_pts_from_depot)
                    sp1 -= 1
                    break
            for idx, value in enumerate(ordered_pts_rest):
                if value == loc_pair[0]:
                    for i in range(ordered_pts_rest.index(value), len(ordered_pts_rest)):
                        if i == idx + key[2]:
                            sp2 += 1
                            itm_stp_2 = value
                            itm_stp_2_dist += ugv_distance_calc_auto.distance_btw_ugvstops(ordered_pts_rest[i], itm_stp_2, ordered_pts_rest)
                            next_loc = perch_and_move_agents([itm_stp_2], sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest)
                            end_loc_id = ordered_pts_rest.index(next_loc[0]) + key[2]
                            if end_loc_id >= len(ordered_pts_rest):
                                end_loc = ordered_pts_rest[-1]
                            else:
                                end_loc = ordered_pts_rest[end_loc_id]
                            itm_stp_1_dist += ugv_distance_calc_auto.distance_btw_ugvstops(next_loc[0], end_loc, ordered_pts_rest)
                            sp2 -= 1
                            break
            for idx, value in enumerate(reversed_ordered_pts_rest):
                if value == loc_pair[1]:
                    for i in range(reversed_ordered_pts_rest.index(value), len(reversed_ordered_pts_rest)):
                        if i == idx + key[3]:
                            sp3 += 1
                            itm_stp_3 = value
                            itm_stp_3_dist += ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ordered_pts_rest[i], itm_stp_3, ordered_pts_rest)
                            next_loc = perch_and_move_agents([itm_stp_3], sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest)
                            end_loc_id = reversed_ordered_pts_rest.index(next_loc[0]) + key[3]
                            if end_loc_id >= len(reversed_ordered_pts_rest):
                                end_loc = reversed_ordered_pts_rest[-1]
                            else:
                                end_loc = reversed_ordered_pts_rest[end_loc_id]
                            itm_stp_3_dist += ugv_distance_calc_auto.distance_btw_ugvstops(next_loc[0], end_loc, ordered_pts_rest)
                            sp3 -= 1
                            break
            key = [ugv_ends_dist, itm_stp_1_dist, itm_stp_2_dist, itm_stp_3_dist]
            # print(f"Updated key list for clustering analysis is: {key}")
            X.append(key)
            if val > 1_000_000:
                y.append(0)
            else:
                y.append(1)
            # for l in sp_feas_list:
            #     y.append(l)
            cnt += 1
        return X, y
    elif isinstance(population, list):
        temp_x_list = []
        for key in population:
            loc_pair = ugv_stops_dict[key[0]]
            count_1 = None
            count_2 = None
            data_flag = 0
            for i, list_val in enumerate(ordered_pts_from_depot):
                if ordered_pts_from_depot[i] == (round(strt_pt[0][0], 2), round(strt_pt[0][1], 2)):
                    count_1 = i
                if ordered_pts_from_depot[i] == (round(loc_pair[0][0], 2), round(loc_pair[0][1], 2)):
                    count_2 = i
            if count_1 or count_2 is None:
                ordered_pts_from_depot = ordered_pts_from_depot_second
                ordered_pts_rest = list(reversed(ordered_pts_rest))
                data_flag = 1
            if data_flag == 1:
                ordered_pts_from_depot = ordered_pts_from_depot_second
                ordered_pts_rest = list(reversed(ordered_pts_rest))
            ugv_ends_dist = ugv_distance_calc_auto.distance_btw_ugvstops(loc_pair[0], loc_pair[1], ordered_pts_rest)
            itm_stp_1_dist = 0
            itm_stp_2_dist = 0
            itm_stp_3_dist = 0
            sp1 = 0
            sp2 = 0
            sp3 = 0
            for idx, value in enumerate(ordered_pts_from_depot):
                if idx == key[1]:
                    sp1 += 1
                    itm_stp_1 = value
                    itm_stp_1_dist += ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(strt_pt[0], itm_stp_1, ordered_pts_from_depot)
                    next_loc = perch_and_move_agents([itm_stp_1], sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest)
                    end_loc_id = ordered_pts_from_depot.index(next_loc[0]) + key[1]
                    if end_loc_id >= len(ordered_pts_from_depot):
                        end_loc = ordered_pts_from_depot[-1]
                    else:
                        end_loc = ordered_pts_from_depot[end_loc_id]
                    itm_stp_1_dist += ugv_distance_calc_auto.distance_from_depotB_btw_ugvstops(next_loc[0], end_loc, ordered_pts_from_depot)
                    sp1 -= 1
                    break
            for idx, value in enumerate(ordered_pts_rest):
                if value == loc_pair[0]:
                    for i in range(ordered_pts_rest.index(value), len(ordered_pts_rest)):
                        if i == idx + key[2]:
                            sp2 += 1
                            itm_stp_2 = value
                            itm_stp_2_dist += ugv_distance_calc_auto.distance_btw_ugvstops(ordered_pts_rest[i], itm_stp_2, ordered_pts_rest)
                            next_loc = perch_and_move_agents([itm_stp_2], sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest)
                            end_loc_id = ordered_pts_rest.index(next_loc[0]) + key[2]
                            if end_loc_id >= len(ordered_pts_rest):
                                end_loc = ordered_pts_rest[-1]
                            else:
                                end_loc = ordered_pts_rest[end_loc_id]
                            itm_stp_1_dist += ugv_distance_calc_auto.distance_btw_ugvstops(next_loc[0], end_loc, ordered_pts_rest)
                            sp2 -= 1
                            break
            for idx, value in enumerate(reversed_ordered_pts_rest):
                if value == loc_pair[1]:
                    for i in range(reversed_ordered_pts_rest.index(value), len(reversed_ordered_pts_rest)):
                        if i == idx + key[3]:
                            sp3 += 1
                            itm_stp_3 = value
                            itm_stp_3_dist += ugv_distance_calc_auto.distance_btw_ugvstops(reversed_ordered_pts_rest[i], itm_stp_3, ordered_pts_rest)
                            next_loc = perch_and_move_agents([itm_stp_3], sp1, sp2, sp3, ugv_velocity, data_flag, ordered_pts_from_depot, ordered_pts_rest)
                            end_loc_id = reversed_ordered_pts_rest.index(next_loc[0]) + key[3]
                            if end_loc_id >= len(reversed_ordered_pts_rest):
                                end_loc = reversed_ordered_pts_rest[-1]
                            else:
                                end_loc = reversed_ordered_pts_rest[end_loc_id]
                            itm_stp_3_dist += ugv_distance_calc_auto.distance_btw_ugvstops(next_loc[0], end_loc, ordered_pts_rest)
                            sp3 -= 1
                            break
            key = [ugv_ends_dist, itm_stp_1_dist, itm_stp_2_dist, itm_stp_3_dist]
            # print(f"Updated key list for clustering analysis is: {key}")
            temp_x_list.append(key)
            cnt += 1
        # computed_list = [temp_x_list[i] for i in range(len(X) - len(population), len(X))]
        return temp_x_list


def clustering_algorithm_each_param_as_dist(population_dict):
    X, y = param_to_distance_mapping(population_dict, ordered_pts_from_depot, ordered_pts_rest)
    scaler = MinMaxScaler()
    X_scaled = scaler.fit_transform(X)
    # kmeans = KMeans(n_clusters=3, random_state=0)
    X_arr = np.array(X)
    X_arr_scaled = np.array(X_scaled)
    y_arr = np.array(y)
    y_arr = y_arr.reshape(-1, 1)
    knn = KNeighborsClassifier()
    y_arr = y_arr.ravel()
    knn.fit(X_arr, y_arr)
    return knn


def clustering_algorithm_each_param_as_dist_ensemble(population_dict):
    X, y = param_to_distance_mapping(population_dict, ordered_pts_from_depot, ordered_pts_rest)
    scaler = MinMaxScaler()
    X_scaled = scaler.fit_transform(X)
    X_arr = np.array(X)
    X_arr_scaled = np.array(X_scaled)
    y_arr = np.array(y)
    y_arr = y_arr.reshape(-1, 1)
    sigma = 0.05  # Noise of observations
    # squared exponential kernel:
    l = np.mean(np.std(X_arr))
    input_dim = len(X_arr[0])
    # gaussian_process_classifier = CustomGaussianProcessClassifier(kernel=GPy.kern.RBF(input_dim=input_dim, lengthscale=l, variance=sigma ** 2))
    # base_learners = [
    #     ('gaussianprocess', gaussian_process_classifier),
    #     ('kNN', KNeighborsClassifier()),
    #     ('svm', SVC(kernel='rbf', probability=True))
    # ]
    base_learners = [
        ('decision_tree', DecisionTreeClassifier()),
        ('kNN', KNeighborsClassifier()),
        ('svm', SVC(kernel='rbf', probability=True))
    ]
    meta_classifier = LogisticRegression()
    stacking_classifier = StackingClassifier(estimators=base_learners, final_estimator=meta_classifier)
    y_arr = y_arr.ravel()
    stacking_classifier.fit(X_arr, y_arr)
    return stacking_classifier


# def gp_classifier(population_dict):
#     X, y = param_to_distance_mapping(population_dict, ordered_pts_from_depot, ordered_pts_rest)
#     X_arr = np.array(X)
#     y_arr = np.array(y)
#     y_arr = y_arr.reshape(-1, 1)
#     m = gp.gp_classification(X_arr, y_arr)
#     return m


def local_minimum(X, num_uavs, ugv_stops_dict, improvement_process):
    global total_local_opt_fn_eval
    minimum = minimize(fun=PS.persistent_surveillance_for_local_improver_agent, x0=X, args=(num_uavs, ugv_stops_dict, improvement_process), method='Nelder-Mead', callback=cb, options={'maxfev': 10}, tol=0.5)
    total_local_opt_fn_eval += len(improvement_process)
    print(f"Total local solutions evaluated in current iteration is: {len(improvement_process)}")
    # Apply destroyer agent for 1st improver agent (local) -> Get unique solutions
    improvement_process = unique(improvement_process)
    print(f"Local population evaluation is: {improvement_process}")
    imp_0, imp_1, imp_2, imp_3 = minimum.x
    for key, val in improvement_process.items():
        key = tuple(key)
        if val > 1_000_000:
            continue
        loc_population_dict[(int(key[0]), int(key[1]), int(key[2]), int(key[3]))] = int(val)
    imp_fitness_val, imp_max_time, imp_depotb_vel, imp_ugv_vel, imp_track_node_age, imp_route_dict, imp_route_dict2, imp_route_dict3, imp_sp_feas_list, ugv_ordered_list, imp_total_cost = PS.persistent_surveillance([int(imp_0), int(imp_1), int(imp_2), int(imp_3)], num_uavs, ugv_stops_dict)
    loc_population_dict[tuple([int(imp_0), int(imp_1), int(imp_2), int(imp_3)])] = imp_total_cost
    print("++++++++++++++++++++++++++++++ Local optimization has ended +++++++++++++++++++++++++++++++")
    return loc_population_dict


def global_minimum(X_list, num_uavs, stops_comb_len, normalized_SA, clustering, label):
    global total_global_opt_fn_eval
    print(f"The label that should not be computed: {label}")
    # evolution_gen_update = X_list
    evolution_gen_update = unique_evolution(X_list)
    # Add the following lines for performing replanning
    if len(evolution_gen_update) < 4:
        for i in range(6 - len(evolution_gen_update)):
            evolution_gen_update.append(X_list[-1-i])
    print("The actual list is: {}".format(X_list))
    print("Updated evolution list for evolution gen is: {}".format(evolution_gen_update))
    evolution_gen = ga.operator_selection(evolution_gen_update, stops_comb_len, normalized_SA)
    print("++++++++++++++++++++++++++++++ Global optimization has started +++++++++++++++++++++++++++++++")
    eliminated_sol_count = 0
    elim_redundant_sol_cnt = 0
    computed_cnt = 0
    y_current_gen = []
    evolution_gen_as_dist = param_to_distance_mapping(evolution_gen, ordered_pts_from_depot, ordered_pts_rest)
    evolution_gen_arr = np.array(evolution_gen_as_dist)
    file_path = "Predictor agent classification plot scenario 2/X_ensembleGP_val_10_scn2_"+str(ga_count)+".csv"  # Change this name while varying the classifier
    np.savetxt(file_path, evolution_gen_arr, delimiter=',', fmt='%d')
    # evolution_gen_arr = evolution_gen_arr.reshape(1, len(evolution_gen_arr))
    scaler = MinMaxScaler()
    evolution_gen_arr_scaled = scaler.fit_transform(evolution_gen_arr)

    '''Classifier using k-NN or ensemble learning'''
    predicted_cluster_list = clustering.predict(evolution_gen_arr)
    predicted_cluster_list_ls = predicted_cluster_list.tolist()
    # predicted_cluster_list = []
    # predict_prob_cluster_list = clustering.predict_proba(evolution_gen_arr)
    # for infeas, feas in predict_prob_cluster_list:
    #     # Threshold for class 0 (feasible)
    #     threshold_feasible = 0.5
    #     # Threshold for class 1 (infeasible)
    #     threshold_infeasible = 0.6
    #     # Classify based on the maximum probability
    #     if feas > infeas:
    #         if feas > threshold_feasible:
    #             predicted_cluster_list.append(1)
    #         else:
    #             predicted_cluster_list.append(0)
    #     elif infeas > feas:
    #         if infeas > threshold_infeasible:
    #             predicted_cluster_list.append(0)
    #         else:
    #             predicted_cluster_list.append(1)
    # predicted_cluster_list_alternate = clustering.predict(evolution_gen_arr)
    # print(predicted_cluster_list_alternate)
    # # predicted_cluster_list_alternate = predicted_cluster_list.tolist()
    # predicted_cluster_list_ls = predicted_cluster_list
    # predicted_cluster_list = np.array(predicted_cluster_list_ls)

    '''Classifier using Gaussian Process'''
    # # making predictions for the test values:
    # mu, var = clustering.predict(evolution_gen_arr)
    #
    # # convert to 1D lists
    # predicted_cluster_list = mu[:, 0]
    # print(f"The predicted output from Gaussian Process is: {predicted_cluster_list}")

    # make output binary data
    # for i in range(len(predicted_cluster_list)):
    #     if predicted_cluster_list[i] > 0.6:
    #         predicted_cluster_list[i] = 1
    #     else:
    #         predicted_cluster_list[i] = 0
    # predicted_cluster_list_ls = predicted_cluster_list.tolist()

    file_path_y = "Predictor agent classification plot scenario 2/y_ensemble_val_10_scn2_"+str(ga_count)+".csv"   # Change this name while varying the classifier
    np.savetxt(file_path_y, predicted_cluster_list, delimiter=',', fmt='%d')

    # Random classifier
    # predicted_cluster_list = [random.choice([0, 1]) for _ in range(len(evolution_gen_arr))]
    # predicted_cluster_list_ls = predicted_cluster_list

    with open('output_10_scn2_ensemble_'+str(ga_count)+'.txt', 'w') as file:
        # Write data for each row
        for i in range(len(evolution_gen)):
            row = evolution_gen[i]
            other_value = predicted_cluster_list[i]
            row_str = '\t'.join(str(x) for x in row)
            file.write(f"{row_str}\t{other_value}\n")
    print(f"predicted_cluster_list is: {predicted_cluster_list} and its length is: {len(predicted_cluster_list)}")
    discard_list = []
    discard_list_id = []
    for i, val in enumerate(predicted_cluster_list):
        if val in label:
            discard_list.append(evolution_gen_as_dist[i])
            discard_list_id.append(i)
    print(f"Total solutions eliminated by clustering: {len(discard_list)}")
    print(f"Discard list id: {discard_list_id}")
    evolution_gen_stlistd = []
    for i in evolution_gen:
        idx = evolution_gen.index(i)
        if predicted_cluster_list_ls[idx] != 0:
            evolution_gen_stlistd.append(i)
    # evolution_gen = [i for i in evolution_gen if evolution_gen.index(i) not in discard_list_id]
    evolution_gen_curr = np.array(evolution_gen)
    idx_to_pop = []
    redundant_id = []
    discarded_outputs = []
    for i, sol in enumerate(evolution_gen_stlistd):
        # sol_arr = np.array(sol)
        # sol_arr = sol_arr.reshape(1, len(sol_arr))
        # print(sol_arr)
        # sol_arr_scaled = StandardScaler().fit_transform(sol_arr)
        # predicted_cluster = kmeans.predict(sol_arr_scaled)
        # print(f'predicted_cluster: {predicted_cluster}')
        if tuple(sol) in population_dict.keys():
            print(f"Predicted cluster is: {predicted_cluster_list[i]}")
            elim_redundant_sol_cnt += 1
            idx_to_pop.append(sol)
            redundant_id.append(i)
            valus = population_dict[tuple(sol)]
            if valus > 1_000_000:
                feas = 0
            else:
                feas = 1
            # discarded_outputs.append(feas)
            y_current_gen.append(feas)
            continue
        print(f'The UGV stop locations for global optimization are: {ugv_stops_dict[sol[0]][0], ugv_stops_dict[sol[0]][1]} and the UGV wait times at each stops are: {sol[1], sol[2]}')
        global_f, global_t, global_depotvel, global_ugvvel, global_track_node_age, global_route_dict, global_route_dict2, global_route_dict3, global_sp_feas_list, ugv_ordered_list, global_total_cost = PS.persistent_surveillance([sol[0], sol[1], sol[2], sol[3]], num_uavs, ugv_stops_dict)
        computed_cnt += 1
        # Apply destroyer agent for 2nd improver agent (global): -> One agent works on the intermediate results of another.
        X.append(sol)
        if global_total_cost > 1_000_000:
            y_current_gen.append(0)
            y.append(0)
        else:
            y_current_gen.append(1)
            y.append(1)
            # for i in global_sp_feas_list:
            #     y.append(i)
        temp = 0
        temp1 = 0
        temp2 = 0
        route_track_dict = {'sp1': [], 'sp2': [], 'sp3': []}
        if route_dict is None:
            temp_dist = 0
            route_track_dict['sp1'].append(temp_dist)
        else:
            for i, val in enumerate(route_dict):
                temp += val[1]
                temp_dist = uav_speed * temp
                route_track_dict['sp1'].append(temp_dist)
        if route_dict2 is None:
            temp_dist = 0
            route_track_dict['sp2'].append(temp_dist)
        else:
            for i, val in enumerate(route_dict2):
                temp1 += val[1]
                temp_dist = uav_speed * temp1
                route_track_dict['sp2'].append(temp_dist)
        if route_dict3 is None:
            temp_dist = 0
            route_track_dict['sp3'].append(temp_dist)
        else:
            for i, val in enumerate(route_dict3):
                temp2 += val[1]
                temp_dist = uav_speed * temp2
                route_track_dict['sp3'].append(temp_dist)
        print(f"Route_track_dict: {route_track_dict}")
        if global_total_cost > 500_000_000:
            overall_evaluated_solutions[tuple(sol)] = global_total_cost
            continue
        glob_population_dict[tuple(sol)] = global_total_cost
    for i, val in enumerate(predicted_cluster_list_ls):
        if i not in discard_list_id:
            if val != 0:
                discarded_outputs.append(predicted_cluster_list_ls[i])
    # discarded_outputs = [i for i in predicted_cluster_list_ls if predicted_cluster_list_ls.index(i) not in discard_list_id]
    print(f"The predicted classified output values are: {discarded_outputs}")
    print(f"The actual classified output values are: {y_current_gen}")
    evolution_gen_reduced = [i for i in evolution_gen_stlistd if i not in idx_to_pop]
    evolution_gen_curr = np.array(evolution_gen_reduced)
    y_current_gen_arr = np.array(y_current_gen)
    y_current_gen_arr = y_current_gen_arr
    discarded_outputs_arr = np.array(discarded_outputs)
    X_arr = np.array(X)
    y_arr = np.array(y).reshape(-1, 1)
    print(f"Global population dictionary is: {glob_population_dict}")
    print(f"Total solutions eliminated by redundancy: {elim_redundant_sol_cnt}")
    print(f"Total solutions computed: {computed_cnt}")
    print(f"Total solutions in this evolution: {len(evolution_gen_stlistd)}")
    total_global_opt_fn_eval += computed_cnt
    print(f"The accuracy score for predicted vs actual responses is: {accuracy_score(y_current_gen_arr, discarded_outputs_arr)*100}%")
    # print(f"Accuracy score for the clustering is: {clustering.score(evolution_gen_curr, y_current_gen_arr)}")
    # print(f"Overall accuracy score for the collected data: {clustering.score(X_arr, y_arr)}")
    print("++++++++++++++++++++++++++++++ Global optimization has ended +++++++++++++++++++++++++++++++")
    return glob_population_dict

# dum_list = {tuple([58, 25, 10]): 2354, tuple([56, 25, 10]): 2354, tuple([55, 26, 9]): 2354, tuple([54, 24, 10]): 2354, tuple([55, 25, 10]): 2354, tuple([56, 26, 9]): 2354, tuple([56, 26, 9]): 2354, tuple([56, 26, 9]): 2354}
# unique_dict = unique(dum_list)


"""Fixed parameters"""
ugv_max_capacity = 25_010_000  # in J
ugv_velocity = 15  # in ft/s
uav_speed = 33  # in ft/s
strt_pt = [(0, 0)]

# Agent descriptions
"""Constructor components:
    Constructor requestors: Request to run only once or runs until a feasible solution is obtained.
    Constructors use null selectors.
    Operators: The algorithm for solution constructor: -> In this case it is random LHS sampling.
    Distributor: Adds the solution to the population. In case of constructor, it is just the random solutions obtained
    from sampling.
"""


t0 = time.time()
np.random.seed(10)
print("----------------Random seed = 10 -------------------")
print("Ensure that the changes that you make here is properly reflected in the 'genetic_algorithm' module")
population_dict = {}
loc_population_dict = {}
glob_population_dict = {}
overall_population_dict = []
overall_conv_crita_list = []
total_global_opt_fn_eval = 0
total_local_opt_fn_eval = 0
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
ugv_Stops = minimum_set_cover_UGV_hyperparam_refined.main(ordered_pts_rest_df, 'Case_Study_scenario6.csv')

for i in range(len(ugv_Stops)):
    ugv_stops_dict[i+2] = ugv_Stops[i]


# Updated UGV end pairs for replanning
# ugv_stops_dict = {}
# ugv_Stops = [[(3.43, 6.9), (7.45, 3.86)], [(3.43, 6.9), (7.62, 3.69)], [(3.43, 6.9), (7.7, 3.61)], [(3.43, 6.9), (8.04, 3.28)]]

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
overall_evaluated_solutions = {}
x_copy = copy.deepcopy(x)
switching_phase_solution = []
sorted_initial_population = x
ga_count = 0
X = []
y = []
y_class = []
obj_val_list = []
duration_dict = {0: ['NW_stop', 'SE_stop', 'NW_TD_1', 'SE_TD', 'Max_time', 'Objective_value', 'total_dist', 'total_time', 'depotb_vel', 'ugvstop_vel', 'ugv_pair_stps', 'start point position']}
for n in range(len(sorted_initial_population)):
    temp_list = []
    param_list = sorted_initial_population.pop(0)
    print("#################################################################################################")
    print(f'The UGV stop locations are: {ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1]} and the UGV wait times at each stops are: {param_list[1], param_list[2]}')
    duration_dict[n+1] = [ugv_stops_dict[param_list[0]][0], ugv_stops_dict[param_list[0]][1], param_list[1], param_list[2], param_list[3]]
    fitness_val, max_time, depotb_vel, ugv_vel, track_node_age, route_dict, route_dict2, route_dict3, sp_feas_list, ugv_ordered_list, total_cost = PS.persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
    temp = 0
    temp1 = 0
    temp2 = 0
    route_track_dict = {'sp1': [], 'sp2': [], 'sp3': []}
    if route_dict is None:
        temp_dist = 0
        route_track_dict['sp1'].append(temp_dist)
    else:
        for i, val in enumerate(route_dict):
            temp += val[1]
            temp_dist = uav_speed * temp
            route_track_dict['sp1'].append(temp_dist)
    if route_dict2 is None:
        temp_dist = 0
        route_track_dict['sp2'].append(temp_dist)
    else:
        for i, val in enumerate(route_dict2):
            temp1 += val[1]
            temp_dist = uav_speed * temp1
            route_track_dict['sp2'].append(temp_dist)
    if route_dict3 is None:
        temp_dist = 0
        route_track_dict['sp3'].append(temp_dist)
    else:
        for i, val in enumerate(route_dict3):
            temp2 += val[1]
            temp_dist = uav_speed * temp2
            route_track_dict['sp3'].append(temp_dist)
    print(f"Route_track_dict: {route_track_dict}")
    population_dict[tuple(param_list)] = total_cost
    overall_evaluated_solutions[tuple(param_list)] = total_cost
    # temp_list.append(total_cost)
    # for m in sp_feas_list:
    #     temp_list.append(m)
    obj_val_list.append(total_cost)
    param_list.append(total_cost)
    if total_cost <= 1_000_000:  # Even though initial popn. is randomized, this framework allows to perform local min even if the solution is infeasible but close to feasibility.
        switching_phase_solution.append(param_list)
        # if len(population_dict) <= 4:
        #     continue
        # else:
        #     break

knn = KNeighborsClassifier()
sorted_initial_population_arr = np.array(x_copy)
obj_val_list_class = []
for i in obj_val_list:
    if i > 1_000_000:
        obj_val_list_class.append(0)
        # for j in i[1:4]:
        #     obj_val_list_class.append(j)
    else:
        obj_val_list_class.append(1)
        # for j in i[1:4]:
        #     obj_val_list_class.append(j)
y_curr_arr = np.array(obj_val_list_class)
y_curr_arr = y_curr_arr.reshape(-1, 1)
y_curr_arr = y_curr_arr.ravel()
# knn.fit(sorted_initial_population_arr, y_curr_arr)
# print(f"Accuracy score for the clustering is: {knn.score(sorted_initial_population_arr, y_curr_arr)}")
pop_keys = list(population_dict.keys())
length = pop_keys[0]
improvement_process = {}
q1 = Queue()
q2 = Queue()
q3 = Queue()
# For first iteration
# flag = 0
# For replanning
flag = 1
iteration_num = 0
convergence_dict = {}
count = 0
normalized_SA = [1, 1, 1, 1]
conv_count = 0
# Switch from constructor phase to improver phase
"""Improver components:
Improver requestor: Requests to start running the improver agents and the improver filters request to run
solutions that are feasible and filters the other infeasible solutions.
Improver selectors: Modify the solutions for improvement.
Improver operator: Local and global optimization algorithms
Improver distributor: Adds improved solutions to output population 'population_dict'.
"""
# Initially apply local minimization algorithm (Improver agent):


def wrapper1(func, arg1, arg2, arg3, arg4, queue):
    queue.put(func(arg1, arg2, arg3, arg4))


def wrapper2(func, arg1, arg2, arg3, arg4, arg5, arg6, queue):
    queue.put(func(arg1, arg2, arg3, arg4, arg5, arg6))


while flag == 1:
    ga_count += 1
    Nfeval = 1
    fout = open('NM_steps_'+'.txt', 'w')
    # if count == 0:
    #     switch_sol_array = np.asarray(switching_phase_solution.pop(0))
    # else:
    #     switch_sol_array = np.asarray(switching_phase_solution.pop())
    switching_phase_solution.sort(key=lambda switching_phase_solution: (switching_phase_solution[4]))
    switch_sol_array = switching_phase_solution.pop(0)
    switch_sol_array = switch_sol_array[:-1]
    # for key, val in population_dict.items():
    #     if val > 1_000_000:
    #         count += 1
    # if count == 0:
    #     flag = 0
    #     break
    evolution_with_obj_val = []
    evolution_without_obj_val = []
    for i, j in population_dict.items():
        evolution_with_obj_val.append([i[0], i[1], i[2], i[3], population_dict[i]])
        evolution_without_obj_val.append([i[0], i[1], i[2], i[3]])

    # Apply local and global optimization algorithm asynchronously/in parallel on the selected solutions (Improver agent):
    evolution_with_obj_val.sort(key=lambda evolution_with_obj_val: (evolution_with_obj_val[4]))
    filtered_data_for_SA = []
    print(len(x_copy))
    # kmeans, cluster_dict = clustering_algorithm_each_param_as_dist(population_dict)
    # knn = clustering_algorithm_each_param_as_dist(overall_evaluated_solutions)
    ensemble = clustering_algorithm_each_param_as_dist_ensemble(overall_evaluated_solutions)
    # gaussian_process = gp_classifier(overall_evaluated_solutions)
    # cluster_list = cluster_list.values.tolist()
    # cluster_list = [cluster_list[i] for i in range(len(cluster_list) - len(population_dict), len(cluster_list))]
    # print(f'Cluster list is: {cluster_list} and its length is: {len(cluster_list)}')
    # cluster_dict = {0: [], 1: [], 2: []}
    # for i, val in enumerate(cluster_list):
    #     if i > len(population_dict):
    #         continue
    #     if val == 0:
    #         cluster_dict[0].append(list(population_dict.values())[i])
    #     elif val == 1:
    #         cluster_dict[1].append(list(population_dict.values())[i])
    #     elif val == 2:
    #         cluster_dict[2].append(list(population_dict.values())[i])
    # print(f"The clustered solutions are: {cluster_dict}")
    # infeasible_label = []
    # for key, val in cluster_dict.items():
    #     if all(x > 1_000_000 for x in val):
    #         infeasible_label.append(key)
    infeasible_label = [0]
    # morris_sa = MorrisMethod(n_samples, num_uavs)
    # morris_SA_data = morris_sa.morris_data_observation(x_copy, space_list, ugv_stops_dict)
    thread1 = Thread(target=wrapper1, args=(local_minimum, switch_sol_array, num_uavs, ugv_stops_dict, improvement_process, q1)).start()
    thread2 = Thread(target=wrapper2, args=(global_minimum, evolution_with_obj_val, num_uavs, stops_comb_len, normalized_SA, ensemble, infeasible_label, q2)).start()
    # thread3 = Thread(target=wrapper3, args=(morris_sa.morris_sensitivity_analysis, morris_SA_data, x_copy, ugv_stops_dict, space_list, q3)).start()
    # sensitivity_values = q3.get()
    # print("++++++++++++++++++++++++++++++ Sensitivity analysis has ended for this iteration +++++++++++++++++++++++++++++++")
    # print(f'Sensitivity analysis values are: {sensitivity_values}')
    # sensitivity_values_list = sensitivity_values['mu_star'].compressed().tolist()
    # min_value = min(sensitivity_values_list)
    # max_value = max(sensitivity_values_list)
    # # Normalize the values using min-max normalization
    # normalized_SA = [(value - min_value) / (max_value - min_value) for value in sensitivity_values_list]
    # print(f'Normalized sensitivity estimates: {normalized_SA}')
    loc_population_dict = q1.get()
    glob_population_dict = q2.get()
    # if iteration_num % 2 == 0:
    for key, val in loc_population_dict.items():
        population_dict[key] = val
        overall_evaluated_solutions[key] = val
    for key, val in glob_population_dict.items():
        population_dict[key] = val
        overall_evaluated_solutions[key] = val
    count = 0
    population_dict_copy = copy.deepcopy(population_dict)
    population_dict = dict(sorted(population_dict.items(), key=lambda item: item[1]))
    population_dict_copy = dict(sorted(population_dict_copy.items(), key=lambda item: item[1]))
    overall_population_dict.append(population_dict_copy)
    convergence_dict[iteration_num] = list(population_dict_copy.values())[0]
    convergence_list = []
    convergence_list.append(list(population_dict_copy.values())[0])
    print(f'Convergence dict: {convergence_dict}')
    if len(convergence_dict) >= 2:
        if convergence_dict[iteration_num-1] == convergence_dict[iteration_num]:
            conv_count += 1
    print(f'Convergence count is: {conv_count}')
    iteration_num += 1
    # for key, val in population_dict.items():
    #     if val > 1_000_000:
    #         count += 1
    # if count == 0:
    #     flag = 0
    # each_pop_list = []
    # for i in population_dict_copy:
    #     each_pop_list.append([i[0], i[1], i[2], i[3]])
    # overall_conv_crita_list.append(each_pop_list)
    # print(overall_conv_crita_list)
    # if len(overall_conv_crita_list) > 1:
    #     list_count = 0
    #     last_list = overall_conv_crita_list[-1]
    #     last_before_list = overall_conv_crita_list[-2]
    #     for key in last_list:
    #         if key in last_before_list:
    #             list_count += 1
    #     if list_count == len(population_dict):
    #         flag = 0
    if len(convergence_dict) >= 2 and conv_count >= 1:
        flag = 0
    # if len(overall_population_dict) > 1:
    #     dict_count = 0
    #     last_dict = overall_population_dict[-1]
    #     last_before_dict = overall_population_dict[-2]
    #     for key in last_dict:
    #         if key in last_before_dict and last_dict[key] == last_before_dict[key]:
    #             dict_count += 1
    #     if dict_count == len(population_dict):
    #         flag = 0
    if len(population_dict) > n_samples:
        for _ in range(len(population_dict) - n_samples):
            population_dict.popitem()
    print(population_dict)
    itr = 0
    for key, val in population_dict.items():
        if itr == 0:
            key = list(key)
            key.append(val)
            switching_phase_solution.append(key)
            itr += 1
            break
    glob_population_dict = {}
    count += 1
t1 = time.time() - t0
print("Elapsed time = {}".format(t1))
print(f"Total local optimization evaluations: {total_local_opt_fn_eval}")
print(f"Total global optimization evaluations: {total_global_opt_fn_eval}")
# print("If any issue is faced, just try to add 'recharge_time' variable to the time windows")
iterations = list(convergence_dict.keys())
best_objective = list(convergence_dict.values())
print(f'Overall evaluated solutions are: {overall_evaluated_solutions} and its length is {len(overall_evaluated_solutions)}')

for i in iterations:
    i += 1

plt.plot(iterations, best_objective)
plt.xlabel('Iteration number')
plt.ylabel('Best objective in current iteration')
plt.title('Convergence plot')
plt.show()
