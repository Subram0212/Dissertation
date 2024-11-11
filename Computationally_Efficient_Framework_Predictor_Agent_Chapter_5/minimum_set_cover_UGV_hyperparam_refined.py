import numpy as np
from ortools.sat.python import cp_model
import matplotlib.pyplot as plt
import pandas as pd
from itertools import combinations, permutations, product

import time


N_S  = [1]
UAV_Energy = []
UGV_Energy = []
Mission_Time = []
Point_visited_by_UAV = []
Point_visited_by_UGV = []
UGV_TSP = []
Ext_t = []


# df = 'Case_Study_scenario.csv'


def is_inside(circle_x, circle_y, rad, x, y):
    if (x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y) <= rad * rad:
        # print("Inside")
        return 1
    else:
        # print("Outside")
        return 0


########################### cumulative distance determination #######################################


def modified_CP(df):
    def cumulative_distance_determination (df):
        df = pd.read_csv(df)
        X = df['X']
        Y = df['Y']
        branch1 = []
        branch2 = []
        branch3 = []
        for i in range(0,3):
            branch1.append((X[i],Y[i]))
        for i in range(4,9):
            branch2.append((X[i],Y[i]))
        for i in range(9,len(X)):
            branch3.append((X[i],Y[i]))

        branch1 = [(X[3],Y[3])]+ branch1[::-1]
        branch2 =  [(X[3],Y[3])]+ branch2
        branch3 = [(X[3],Y[3])]+ branch3
        ############# branches ####################
        co = branch1
        co1 = branch2
        co2 = branch3
        ###### location nodes ###########
        A =  co + co1[1:]+ co2[1:]
        ############# cumulative distance determination ###################
        dis=[]
        prv_dis=0
        for _ in range(len(co)-1):
            dist = prv_dis +((co[_][0]-co[_+1][0])**2+(co[_][1]-co[_+1][1])**2)**0.5
            dis.append(dist)
            prv_dis= dist

        dis1=[]
        prv_dis1=0
        for _ in range(len(co1)-1):
            dist1 = prv_dis1 +((co1[_][0]-co1[_+1][0])**2+(co1[_][1]-co1[_+1][1])**2)**0.5
            dis1.append(dist1)
            prv_dis1= dist1

        dis2=[]
        prv_dis2=0
        for _ in range(len(co2)-1):
            dist2 = prv_dis2 +((co2[_][0]-co2[_+1][0])**2+(co2[_][1]-co2[_+1][1])**2)**0.5
            dis2.append(dist2)
            prv_dis2= dist2



        cu_dis_dict={}
        cu_dis_dict[0]=0.0
        for _ in range(1,len(co)):
            cu_dis_dict[_]= dis[_-1]

        cu_dis_dict1={}
        cu_dis_dict1[0]=0.0
        for _ in range(1,len(co1)):
            cu_dis_dict1[_]= dis1[_-1]

        cu_dis_dict2={}
        cu_dis_dict2[0]=0.0
        for _ in range(1,len(co2)):
            cu_dis_dict2[_]= dis2[_-1]





        cu_dis = []
        for j in range(len(co)):
            ran= []
            for i in range(len(co)):
                ran.append(round(abs(cu_dis_dict[i]-cu_dis_dict[j])*5280))
            for i in range(1,len(co1)):
                ran.append(round(abs(cu_dis_dict1[i]+cu_dis_dict[j])*5280))
            for i in range(1,len(co2)):
                ran.append(round(abs(cu_dis_dict2[i]+cu_dis_dict[j])*5280))

            cu_dis.append(ran)





        for j in range(1,len(co1)):
            ran= []
            for i in range(len(co)):
                ran.append(round(abs(cu_dis_dict[i]+cu_dis_dict1[j])*5280))
            for i in range(1,len(co1)):
                ran.append(round(abs(cu_dis_dict1[i]-cu_dis_dict1[j])*5280))
            for i in range(1,len(co2)):
                ran.append(round(abs(cu_dis_dict2[i]+cu_dis_dict1[j])*5280))

            cu_dis.append(ran)

        for j in range(1,len(co2)):
            ran= []
            for i in range(len(co)):
                ran.append(round(abs(cu_dis_dict[i]+cu_dis_dict2[j])*5280))
            for i in range(1,len(co1)):
                ran.append(round(abs(cu_dis_dict1[i]+cu_dis_dict2[j])*5280))
            for i in range(1,len(co2)):
                ran.append(round(abs(cu_dis_dict2[i]-cu_dis_dict2[j])*5280))

            cu_dis.append(ran)

        return cu_dis,A
    cu_dis,A = cumulative_distance_determination(df)

    def cu_dis1(node1,node2):
        ix1 = A.index(node1)
        ix2 = A.index(node2)
        return cu_dis[ix1][ix2]


    class VarArraySolutionPrinter(cp_model.CpSolverSolutionCallback):
        """Print intermediate solutions."""

        def __init__(self, variables):
            cp_model.CpSolverSolutionCallback.__init__(self)
            self.__variables = variables
            self.__solution_count = 0
            self.Sol = []
            self.df = df

        def on_solution_callback(self):
            df = self.df
            Rnd_p = []
            df = pd.read_csv(df)
            X = df['X']
            Y = df['Y']
            Targets=[]
            for i in range(len(X)):
                Targets.append((X[i],Y[i]))
            Fuel_limit= 287700
            A = Targets.copy()
            def hit(point,targets,Fuel_limit):
                hitted_targets = []
                for j in range(len(targets)):
                    a = np.array(point)
                    b = np.array(targets[j])
                    d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                    if (198*d/33) <=(Fuel_limit/2):
                        hitted_targets.append(j)
                return hitted_targets
            starting = [(X[0],Y[0])]
            hit_start= hit(starting[0],Targets,Fuel_limit)
            remv=[]
            for i in hit_start:
                remv.append(Targets[i])


            for i in remv:
                Targets.remove(i)
            A.remove(starting[0])
            self.__solution_count += 1
            for v in self.__variables:
                if self.Value(v) == 1:
                    # print('%s=%i' % (v, self.Value(v)), end=' ')
                    a,b = str(v).split(" ")
                    Rnd_p.append(A[int(b)])
            #print(starting+Rnd_p)
            #  print('\n')
            self.Sol.append(starting+Rnd_p)


        def solution_count(self):
            return self.__solution_count

        def all_sol(self):
            return self.Sol

    def CP_set_cover(df):
        df = pd.read_csv(df)
        X = df['X']
        Y = df['Y']
        branch1 = []
        branch2 = []
        branch3 = []
        for i in range(0,3):
            branch1.append((X[i],Y[i]))
        for i in range(4,9):
            branch2.append((X[i],Y[i]))
        for i in range(9,len(X)):
            branch3.append((X[i],Y[i]))

        branch1 = [(X[3],Y[3])] + branch1[::-1]
        branch2 =  [(X[3],Y[3])] + branch2
        branch3 = [(X[3],Y[3])] + branch3
        E1= branch2[-1]
        E2= branch3[-1]
        S = (X[0],Y[0])
        Targets=[]
        for i in range(len(X)):
            Targets.append((X[i],Y[i]))
        Fuel_limit= 287700
        A = Targets.copy()

        """Making sure what points are covered within the fuel coverage from the starting point"""
        def hit(point,targets,Fuel_limit):
            hitted_targets = []
            for j in range(len(targets)):
                a = np.array(point)
                b = np.array(targets[j])
                d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                if (198*d/33) <= (Fuel_limit/2):
                    hitted_targets.append(j)
            return hitted_targets
        starting = [(X[0],Y[0])]
        hit_start= hit(starting[0],Targets,Fuel_limit)
        remv=[]
        for i in hit_start:
            remv.append(Targets[i])

        """Removing those points that are already covered by the coverage radius from Depot. Rest are targets"""
        for i in remv:
            Targets.remove(i)
        A.remove(starting[0])

        ############## minimum hitting set #################

        model = cp_model.CpModel()
        hitted_dict = {}
        point_vars= {}
        target_set={}
        hit_set={}
        obj_var={}
        objective_set={}
        var = []
        for i in range(len(Targets)):
            target_set.setdefault('Target {}'.format(i), set())
            hit_set.setdefault('Target {}'.format(i), set())
        for i in range(len(A)):
            p_var= point_vars.setdefault('Point {}'.format(i), model.NewBoolVar('Point '+str(i)))
            """This dictionary 'hitted_dict' will capture how many points are hit with each mission point being the center"""
            hitted_dict.setdefault('{}'.format(i), set(hit(A[i],Targets,Fuel_limit)))
            obj_var.setdefault('Point {}'.format(i),(cu_dis1(A[i],starting[0])))

        for a, b in point_vars.items():
            var.append(b)

        for a, b in obj_var.items():

            objective_set.setdefault(a,(point_vars[a]))

        for a, b in hitted_dict.items():
            for k in b:
                hit_set['Target {}'.format(k)].add(a)

        for a, b in hit_set.items():
            for k in b:
                target_set['{}'.format(a)].add(list(point_vars.values())[int(k)])

        for a,b in target_set.items():
            model.Add(sum(b) >= 1)
        model.Minimize(sum(objective_set.values()))
        solver = cp_model.CpSolver()
        status = solver.Solve(model)
        if status == cp_model.OPTIMAL:
            selected_points = [p for p in point_vars
                               if solver.Value(point_vars[p])]
            print('No of Rendezvous points: {}\n' .format((len(selected_points))+1))
        else:
            print('Unable to find an optimal solution')

        model = cp_model.CpModel()
        hitted_dict = {}
        point_vars= {}
        target_set={}
        hit_set={}
        obj_var={}
        objective_set={}
        var = []
        for i in range(len(Targets)):
            target_set.setdefault('Target {}'.format(i), set())
            hit_set.setdefault('Target {}'.format(i), set())
        for i in range(len(A)):
            p_var= point_vars.setdefault('Point {}'.format(i), model.NewBoolVar('Point '+str(i)))

            hitted_dict.setdefault('{}'.format(i), set(hit(A[i],Targets,Fuel_limit)))
            obj_var.setdefault('Point {}'.format(i),(cu_dis1(A[i],starting[0])))

        for a, b in point_vars.items():
            var.append(b)

        for a, b in obj_var.items():

            objective_set.setdefault(a,(point_vars[a]))

        for a, b in hitted_dict.items():
            for k in b:
                hit_set['Target {}'.format(k)].add(a)


        for a, b in hit_set.items():
            for k in b:
                target_set['{}'.format(a)].add(list(point_vars.values())[int(k)])


        for a,b in target_set.items():
            model.Add(sum(b) >= 1)
        model.Add(sum(objective_set.values()) == len(selected_points))
        solution_printer = VarArraySolutionPrinter(var)
        # Enumerate all solutions.
        solver.parameters.enumerate_all_solutions = True
        # Solve.
        status = solver.Solve(model, solution_printer)

        All_sol = solution_printer.all_sol()
        print('Status = %s' % solver.StatusName(status))
        print('Number of solutions found: %i' % solution_printer.solution_count())
        #print('All solutions found: {}'.format(All_sol))
        return All_sol,E1,E2,S


    All_sol,E1,E2,S = CP_set_cover(df)

    return All_sol, E1, E2, S


def f_matrix(set1, set2, scenario_points):
    radius = 24750/5280
    count = 0
    data = scenario_points
    data_list = data.values.tolist()
    for i in range(2):  # this 'i' represents min-max points
        border_list1 = []
        sum_list = []
        circle_x1 = set1[i][0]
        circle_y1 = set1[i][1]
        circle_x2 = set2[-i-1][0]
        circle_y2 = set2[-i-1][1]
        for k in range(len(data_list)):
            # if k <= 15:
            #     continue
            x = data_list[k][0]
            y = data_list[k][1]
            cond1 = is_inside(circle_x1, circle_y1, radius, x, y)
            cond2 = is_inside(circle_x2, circle_y2, radius, x, y)
            border_list1.append([cond1, cond2])
            border = border_list1.pop(0)
            sum_list.append(sum(border))
        if sum_list.count(0) == 0:  # Counting if there's any zeros in the list and check if there's no zeros
            count += 1
    return count


def main(scenario_points, df):
    data = scenario_points
    # df = 'Case_Study_scenario.csv'
    data_list = data.values.tolist()

    data_list = [(round(node[0], 2), round(node[1], 2)) for node in data_list]
    data_list = tuple(data_list)
    solutions, endpt1, endpt2, Startpt = modified_CP(df)
    # print(solutions)
    start_pt = []
    ugv_stops = {}
    len_rendezvous_pts = len(solutions[0])
    for i in range(1, len_rendezvous_pts):
        ugv_stops['Stp_'+str(i)] = []

    for i, val in enumerate(solutions):
        start_pt.append(val[0])
    for i, val in enumerate(solutions):
        for j in range(1, len_rendezvous_pts):
            ugv_stops['Stp_'+str(j)].append((round(solutions[i][j][0], 2), round(solutions[i][j][1], 2)))

    for key, val in ugv_stops.items():
        ugv_stops[key] = set(val)

    # for i in range(len(ugv_stops)):
    #     print('Stp_'+str(i+1)+' hyperparameters: {}'.format(ugv_stops['Stp_'+str(i+1)]))

    # TODO: Should automate it better to accomodate variable UGV stops that are greater or lesser than 2
    ugv_stops_arranged = {}
    for i in range(1, len_rendezvous_pts):
        ugv_stops_arranged['Stp_'+str(i)] = []

    for i in data_list:
        for j in range(1, len_rendezvous_pts):
            if i in ugv_stops['Stp_'+str(j)]:
                ugv_stops_arranged['Stp_'+str(j)].append(i)
    ugv_stops_arranged_list = list(ugv_stops_arranged.values())
    # stops_combo_list = []
    # max_set_count = []
    # for key, val in ugv_stops_arranged.items():
    #     stops_combo_list.append(list(combinations(val, 2)))  # this '2' represents the min-max stop points
    #     max_set_count.append(len(ugv_stops[key]))
    #
    # max_val = max(max_set_count)
    # set_hitting_targets = {}
    # set_hitting_targets_list1 = []
    # set_hitting_targets_list2 = []
    # for i in range(len(stops_combo_list[1])):
    #     stp_set_2 = 0
    #     if i >= len(stops_combo_list[0]):
    #         a = stops_combo_list[0]
    #         idx = np.random.choice(len(a), 1)
    #         stp_set_1 = stops_combo_list[0][idx[0]]
    #     else:
    #         stp_set_1 = stops_combo_list[0][i]
    #     stp_set_2 = stops_combo_list[1][i]
    #     target_hit_count = f_matrix(stp_set_1, stp_set_2)
    #     if target_hit_count >= 2:
    #         set_hitting_targets[i] = [stp_set_1, stp_set_2]
    #         set_hitting_targets_list1.append(stp_set_1[0])
    #         set_hitting_targets_list1.append(stp_set_1[1])
    #         set_hitting_targets_list2.append(stp_set_2[0])
    #         set_hitting_targets_list2.append(stp_set_2[1])
    #
    # # TODO: Must debug this issue of choosing the optimal number of set! (AKA post processing step)
    # set_hitting_targets = set(set_hitting_targets_list1)
    # post_processed_set = set()
    # for i in set_hitting_targets:
    #     if i == (4.29, 11.21) or i == (4.01, 10.83):
    #         continue
    #     else:
    #         post_processed_set.add(i)
    #
    # # print(f'Hyperparameter set for stop 1 after post-processing: {post_processed_set}')
    # # print(f'Hyperparameter set for stop 2 after post-processing: {set(set_hitting_targets_list2)}')
    #
    # ugv_Stops_list = list(product(post_processed_set, set(set_hitting_targets_list2)))
    def calculate_cartesian_product(lists):
        stop_list = list(product(*lists))
        return stop_list
    ugv_Stops_list = calculate_cartesian_product(ugv_stops_arranged_list)
    ugv_Stops_list = [list(i) for i in ugv_Stops_list]
    return ugv_Stops_list


if __name__ == "__main__":
    mission_pts = pd.read_excel('Mission locations hardware - ordered.xlsx', engine='openpyxl')
    res = main(mission_pts, 'Mission locations hardware - scaledup for simulation.csv')
    print(res)
