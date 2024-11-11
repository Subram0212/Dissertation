# import matlab.engine
# from scipy.optimize import fmin, fmin_tnc, minimize
# from inner_loop_optimization import main
# import numpy as np
# #
# eng = matlab.engine.start_matlab()
# #
# # # function = inner_loop_optimization.main(x[0], x[1], x[2])
# # # type(function)
# # # guess_f = [58, 25, 10]
# # # minimum = fmin(func=main, x0=guess_f, ftol=1e-2, xtol=2e-1)
# # # print("The corresponding optimal X values are: {}, {}, {}".format(minimum[0], minimum[1], minimum[2]))
# #
# # guess_f1 = np.array([58, 25, 10])
# # # minimum1 = fmin_tnc(func=main, x0=guess_f1, approx_grad=True)
# # # print("The corresponding optimal X values are: {}, {}, {}".format(minimum1[0], minimum1[1], minimum1[2]))
# #
# # methods = ['Nelder-Mead', 'Powell', 'CG', 'BFGS', 'Newton-CG', 'L-BFGS-B', 'TNC', 'COBYLA', 'SLSQP', 'trust-constr', 'dogleg', 'trust-ncg', 'trust-exact', 'trust-krylov']
# # # for i in range(len(methods)):
# # #     guess_f1 = np.array([58, 25, 10])
# # #     if methods[i] == 'CG' or methods[i] == 'BFGS' or methods[i] == 'Newton-CG' or methods[i] == 'trust-ncg' or methods[i] == 'dogleg' or methods[i] == 'L-BFGS-B' or methods[i] == 'TNC' or methods[i] == 'SLSQP' or methods[i] == 'trust-krylov' or methods[i] == 'trust-exact' or methods[i] == 'trust-constr':
# # #         minimum1 = minimize(fun=main, x0=guess_f1, method=methods[i], jac=True, tol=0.2)
# # #     else:
# # #         minimum1 = minimize(fun=main, x0=guess_f1, method=methods[i], tol=0.2)
# # #     # print("The corresponding optimal X values are: {}, {}, {}".format(minimum1[0], minimum1[1], minimum1[2]))
# #
# # improvement_process = []
# # fun_eval = 1
# #
# #
# # def cb(X):
# #     improvement_process.append([int(X[0]), int(X[1]), int(X[2])])
# #     print(improvement_process)
# #
# #
# # minimum1 = minimize(fun=main, x0=guess_f1, method=methods[0], callback=cb, options={'disp': True}, tol=0.2)
# #
# #
# #
# run_local_search = eng.run_local_search(58, 25)
# print(run_local_search)
# # guess_f1 = [58, 25]
# # minimum1 = eng.fminunc(run_local_search, guess_f1)
# # print("The corresponding optimal X values are: {}, {}, {}".format(minimum1[0], minimum1[1], minimum1[2]))
# #
# # list1 = [2, 3, 4, 5]
# # list2 = np.square(list1)
# # print(list2)
# # print(type(list2))
