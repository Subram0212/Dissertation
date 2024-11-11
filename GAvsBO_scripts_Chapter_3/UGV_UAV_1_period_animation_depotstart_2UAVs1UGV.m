clc
close all
speed_factor = 1;
%% Plot the data points
x = [4.29, 4.01, 3.74, 7.7, 3.42, 3.29, 3.02, 2.61, 2.43, 2.77, 3.1, 3.43, 3.8, 0.6, 0.63, 0.75, 1.2, 1.54, 2.44, 3.37, 3.79, 10.54, 10.44, 10.28, 10.58, 10.75, 10.33, 9.93, 9.53, 9.13, 8.75, 8.39, 8.04, 7.7, 7.62, 7.45, 7.11, 6.77, 6.43, 6.22, 6.09, 5.77, 5.37, 4.98, 4.66, 4.55, 4.11, 3.79, 3.54];
y = [11.21, 10.83, 10.45, 3.61, 9.61, 9.23, 8.87, 8.34, 7.91, 7.57, 7.23, 6.9, 6.71, 8.07, 7.97, 7.58, 6.82, 6.48, 6.46, 6.33, 6.71, 0.9, 0.96, 1.28, 1.58, 1.79, 1.88, 2.09, 2.29, 2.5, 2.73, 2.98, 3.28, 3.61, 3.69, 3.86, 4.19, 4.53, 4.86, 5.25, 5.64, 5.87, 6.07, 6.28, 6.4, 6.44, 6.59, 6.71, 10.04];
for i = 1:length(x)
    if x(i) == 0.6 && y(i) == 8.07
        plot(x(i), y(i), 'ko', 'Linewidth', 6); hold on
    else
        plot(x(i), y(i), 'kx', 'LineWidth', 2); hold on
    end
end
text(0.6, 8.07, 'Depot B', 'Color', 'Black', 'FontSize', 10);
text(0.8, 9, 'Start', 'Color', 'Black', 'FontSize', 10);
%% Read the excel data for animation
full_table = readtable('Interpolated_data_UIC_UMD_planner_results_2UAVs1UGV.xlsx');
x1_tab_val = full_table.x;
x2_tab_val = full_table.x_1;
x3_tab_val = full_table.x_2;

y1_tab_val = full_table.y;
y2_tab_val = full_table.y_1;
y3_tab_val = full_table.y_2;

acu_x1 = transpose(x1_tab_val);
acu_x2 = transpose(x2_tab_val);
acu_x3 = transpose(x3_tab_val);

acu_y1 = transpose(y1_tab_val);
acu_y2 = transpose(y2_tab_val);
acu_y3 = transpose(y3_tab_val);
%% Create movie and start animation
mov = VideoWriter('UAV_UGV_animation_2UAVs1UGV_UIC_UMD_colab.avi');
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
  uav_2 = plot(acu_x2(k), acu_y2(k), 'go', 'LineWidth', 2);
  h2 = text(9, 10, 'o - UAV 2','Color','green','FontSize',10);
  ugv = plot(acu_x3(k), acu_y3(k), 'bo', 'LineWidth', 6);
  h3 = text(9, 9,'O - UGV','Color','blue','FontSize',10);
  if (k == 1 || k == 10 || k == 20 || k == 38 || k == 39 || k == 49 || k == 54 || k == 65 || k == 70 || k == 82 || k == 90 || k == 105 || k == 110)
      x1 = plot(acu_x1(k), acu_y1(k), 'rx', 'Linewidth', 2);
  end
  if (k == 1 || k == 12 || k == 22 || k == 47 || k == 48 || k == 59 || k == 77 || k == 82 || k == 86)
      x1 = plot(acu_x2(k), acu_y2(k), 'gx', 'Linewidth', 2);
  end
  if (k == 1 || k == 5 || k == 9 || k == 13 || k == 17 || k == 21 || k == 25 || k == 29 || k == 32 || k == 35 || k == 94 || k == 97 || k == 100 || k == 103 || k == 106 || k == 109 || k == 112 || k == 115 || k == 118 || k == 120 || k == 123 || k == 126 || k == 129 || k == 132 || k == 135 || k == 138 || k == 141 || k == 144 || k == 146 || k == 149 || k == 151 || k == 153 || k == 155 || k == 157 || k == 159 || k == 161 || k == 164)
      x1 = plot(acu_x3(k), acu_y3(k), 'bx', 'Linewidth', 2);
  end

  xlim([min(x(:)), max(x(:))]);
  ylim([min(y(:)), max(y(:))]);
  writeVideo(mov,getframe);
  pause(0.1*speed_factor)

  h = [uav_1, uav_2, ugv];
  delete(h)
  del = [x0, z0, d];
  delete(del)
  
  if (exist('y2','var')), delete(y2); end
  if (exist('time','var')), delete(time); end

end
%% Close the movie
close(mov);