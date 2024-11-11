%% Plot the data points
t = readtable('ARL corridor cov_scn limited data points.xlsx');
x_trans = t(:,1);
y_trans = t(:,2);
x_vec = table2array(x_trans);
x = transpose(x_vec);
y_vec = table2array(y_trans);
y = transpose(y_vec);
save('Corridor scenario.mat');
data = load('Corridor scenario.mat');
for i = 1:length(x)
    if x(i) == 0.6 && y(i) == 8.07
        plot(x(i), y(i), 'ko', 'Linewidth', 6); hold on
    else
        plot(x(i), y(i), 'kx', 'LineWidth', 2); hold on
    end
end

%%