init_sol = [2, 1];
% save parms.mat
% output = py.inner_loop_optimization.main(parms);
fun = py.trial_function.main()
% fun1 = 'fminsearch';

[x, feval] = fminsearch(fun, init_sol)

%% Passing user-defined matlab function into Python
% function f=run_local_search(x, y)
% f = (x-1)^2 + 10*((y - x^2)^2);
% end



