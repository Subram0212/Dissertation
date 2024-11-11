import matlab.engine
from scipy.optimize import fmin, fmin_tnc, minimize
# from inner_loop_optimization import main
import numpy as np

eng = matlab.engine.start_matlab()

# function = inner_loop_optimization.main(x[0], x[1], x[2])
# type(function)
# guess_f = [58, 25, 10]
# minimum = fmin(func=main, x0=guess_f, ftol=1e-2, xtol=2e-1)
# print("The corresponding optimal X values are: {}, {}, {}".format(minimum[0], minimum[1], minimum[2]))

# guess_f1 = [58, 25, 10]
# minimum1 = fmin_tnc(func=main, x0=guess_f1, approx_grad=True)
# print("The corresponding optimal X values are: {}, {}, {}".format(minimum1[0], minimum1[1], minimum1[2]))

# methods = ['Nelder-Mead', 'Powell', 'CG', 'BFGS', 'Newton-CG', 'L-BFGS-B', 'TNC', 'COBYLA', 'SLSQP', 'trust-constr', 'dogleg', 'trust-ncg', 'trust-exact', 'trust-krylov']
# for i in range(len(methods)):
#     guess_f1 = np.array([58, 25, 10])
#     minimum1 = minimize(fun=main, x0=guess_f1, method=methods[i])
#     print("The corresponding optimal X values are: {}, {}, {}".format(minimum1[0], minimum1[1], minimum1[2]))

x = [1, 1]
guess_f1 = matlab.double(x)
# print(x)
# eng.evalc("s = load('parms.mat');")
# myVar1 = eng.eval("s.parms");
# print(myVar1)

# a = matlab.double([1,4,9,16,25])
# b = eng.sqrt(a)
# print(b)


def main(x):
    x1 = x[0]
    x2 = x[1]
    y = 3*np.square(x1) + 2*x1*x2 + np.square(x2) - 4*x1 + 5*x2
    y = matlab.double(y)
    return y


minimum1 = eng.fminunc(main, guess_f1)
# print("The corresponding optimal X values are: {}, {}, {}".format(minimum1[0], minimum1[1], minimum1[2]))

eng.quit()
