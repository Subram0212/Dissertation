clc
close all
speed_factor = 1;
%% Plot the data points
x = [4.29, 4.01, 3.74, 7.7, 3.42, 3.29, 3.02, 2.61, 2.43, 3.1, 3.43, 3.8, 0.6, 0.63, 0.75, 1.2, 1.54, 2.44, 3.37, 3.79, 10.54, 10.44, 10.28, 10.58, 10.75, 10.33, 9.93, 9.53, 9.13, 8.75, 8.39, 8.04, 7.7, 7.62, 7.45, 7.11, 6.77, 6.43, 6.22, 6.09, 5.77, 5.37, 4.98, 4.66, 4.55, 4.11, 3.79, 3.54];
y = [11.21, 10.83, 10.45, 3.61, 9.61, 9.23, 8.87, 8.34, 7.91, 7.23, 6.9, 6.71, 8.07, 7.97, 7.58, 6.82, 6.48, 6.46, 6.33, 6.71, 0.9, 0.96, 1.28, 1.58, 1.79, 1.88, 2.09, 2.29, 2.5, 2.73, 2.98, 3.28, 3.61, 3.69, 3.86, 4.19, 4.53, 4.86, 5.25, 5.64, 5.87, 6.07, 6.28, 6.4, 6.44, 6.59, 6.71, 10.04];
for i = 1:length(x)
    if x(i) == 0.6 && y(i) == 8.07
        plot(x(i), y(i), 'ko', 'Linewidth', 6); hold on
    else
        plot(x(i), y(i), 'kx', 'LineWidth', 2); hold on
    end
end
text(0.6, 8.07, 'Depot B', 'Color', 'Black', 'FontSize', 10);
text(3.79, 7, 'Start', 'Color', 'Black', 'FontSize', 10);
%% Read the excel data for animation
full_table = readtable('Interpolated_data_genetic_algorithm_depotbstrt_ICUAS_22.xlsx');
x1_tab_val = full_table.x;
x2_tab_val = full_table.x_1;

y1_tab_val = full_table.y;
y2_tab_val = full_table.y_1;

acu_x1 = transpose(x1_tab_val);
acu_x2 = transpose(x2_tab_val);

acu_y1 = transpose(y1_tab_val);
acu_y2 = transpose(y2_tab_val);

%% Create movie and start animation
mov = VideoWriter('UAV_UGV_animation_optimized_UGV_params_genetic_algorithm_multistart_3stops.avi');
mov.FrameRate=5;
open(mov);
for k = 1 : length(acu_x1)
    wall_clock_time = sprintf('Wall time elapsed = %d minutes', k);
    time = text(4, 10, wall_clock_time, 'Color', 'Black', 'Fontsize', 10);
    if (k == 1 || k == 2 || k == 3 || k == 4 || k == 5)
      d = plot(acu_x1(k), acu_y1(k), 'ro', 'LineWidth', 2);
      x0 = text(1, 10.5,'Start[0]','Color','red','FontSize',20);
%       y0 = text(1, 10,'Start[0]','Color','magenta','FontSize',20);
      z0 = text(1, 9.5,'Start[0]','Color','blue','FontSize',20);
    end
  uav_1 = plot(acu_x1(k), acu_y1(k), 'ro', 'LineWidth', 2);
  h1 = text(9, 11,'o - UAV 1','Color','red','FontSize',10);
  ugv = plot(acu_x2(k), acu_y2(k), 'bo', 'LineWidth', 6);
  h2 = text(9, 10,'O - UGV','Color','blue','FontSize',10);
  if (k == 1 || k == 15 || k == 18 || k == 21 || k == 24 || k == 28 || k == 36 || k == 57 || k == 58 || k == 60 || k == 61)
      x1 = plot(acu_x1(k), acu_y1(k), 'rx', 'Linewidth', 2);
  end

  xlim([min(x(:)), max(x(:))]);
  ylim([min(y(:)), max(y(:))]);
  writeVideo(mov,getframe);
  pause(0.1*speed_factor)

  h = [uav_1, ugv];
  delete(h)
  del = [x0, z0, d];
  delete(del)
  
  if (exist('y2','var')), delete(y2); end
  if (exist('time','var')), delete(time); end

end
%% Close the movie
close(mov);