clc
close all
speed_factor = 1;
%% Plot the data points
% x = [4.29, 4.01, 3.74, 7.7, 3.42, 3.29, 3.02, 2.61, 2.43, 3.1, 3.43, 3.8, 0.6, 0.63, 0.75, 1.2, 1.54, 2.44, 3.37, 3.79, 10.54, 10.44, 10.28, 10.58, 10.75, 10.33, 9.93, 9.53, 9.13, 8.75, 8.39, 8.04, 7.7, 7.62, 7.45, 7.11, 6.77, 6.43, 6.22, 6.09, 5.77, 5.37, 4.98, 4.66, 4.55, 4.11, 3.79, 3.54];
% y = [11.21, 10.83, 10.45, 3.61, 9.61, 9.23, 8.87, 8.34, 7.91, 7.23, 6.9, 6.71, 8.07, 7.97, 7.58, 6.82, 6.48, 6.46, 6.33, 6.71, 0.9, 0.96, 1.28, 1.58, 1.79, 1.88, 2.09, 2.29, 2.5, 2.73, 2.98, 3.28, 3.61, 3.69, 3.86, 4.19, 4.53, 4.86, 5.25, 5.64, 5.87, 6.07, 6.28, 6.4, 6.44, 6.59, 6.71, 10.04];
t = readtable('Scenario 2 data points.csv');
x_trans = t(:,1);
y_trans = t(:,2);
x_vec = table2array(x_trans);
x = transpose(x_vec);
y_vec = table2array(y_trans);
y = transpose(y_vec);
save('Scenario 2 data points.mat');
data = load('Scenario 2 data points.mat');
for i = 1:length(x)
    if x(i) == 0 && y(i) == 0 || x(i) == 8.66025403784438 && y(i) == 4.99999999999999
        plot(x(i), y(i), 'ko', 'Linewidth', 6); hold on
    else
        plot(x(i), y(i), 'kx', 'LineWidth', 2); hold on
    end
end
text(0.75, 0.35, 'Depot B', 'Color', 'Black', 'FontSize', 10);
text(8, 4.54, 'Start', 'Color', 'Black', 'FontSize', 10);
% text(3.79, 7, 'Start', 'Color', 'Black', 'FontSize', 10);
% text(5, 10, 'Wall time = 643 minutes', 'Color', 'Black', 'Fontsize', 10);
%% Read the excel data for animation
full_table = readtable('Interpolated_data_midpt_optimization_scenario2.xlsx');
x1_tab_val = full_table.x;
x2_tab_val = full_table.x_1;

y1_tab_val = full_table.y;
y2_tab_val = full_table.y_1;

acu_x1 = transpose(x1_tab_val);
acu_x2 = transpose(x2_tab_val);

acu_y1 = transpose(y1_tab_val);
acu_y2 = transpose(y2_tab_val);

%% Create movie and start animation
mov = VideoWriter('UAV_UGV_animation_midpt_optimized_UGV_params_scenario2.avi');
mov.FrameRate=5;
open(mov);
for k = 1 : length(acu_x1)
    wall_clock_time = sprintf('Wall time elapsed = %d minutes', k);
    time = text(1, 8, wall_clock_time, 'Color', 'Black', 'Fontsize', 10);
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
  if (k == 55 || k == 57 || k == 58 || k == 59 || k == 60 || k == 61 || k == 62 || k == 63 || k == 64 || k == 65 || k == 66 || k == 68 || k == 70 || k == 77 || k == 80 || k == 82 || k == 85 || k == 100 || k == 101 || k == 102 || k == 103 || k == 104 || k == 105 || k == 106 || k == 107 || k == 108 || k == 109 || k == 110 || k == 111 || k == 112 || k == 113 || k == 114)
      x1 = plot(acu_x1(k), acu_y1(k), 'rx', 'Linewidth', 2);
  end
%   if (k == 320 || k == 350 || k == 354 || k == 355 || k == 366 || k == 384 || k == 387 || k == 393 || k == 394 || k == 399 || k == 409 || k == 447 || k == 467 || k == 473 || k == 474 || k == 475 || k == 483 || k == 484 || k == 504 || k == 506 || k == 507 || k == 509 || k == 511 || k == 512 || k == 518 || k == 521 || k == 603)
%       plot(acu_x1(k), acu_y1(k), 'gx', 'Linewidth', 5);
%   end
%   if (k == 120)
%       plot(acu_x2(k), acu_y2(k), 'gx', 'Linewidth', 5);
%   end
  if (k == 1 || k == 5 || k == 9 || k == 13 || k == 17 || k == 21 || k == 25 || k == 29 || k == 33 || k == 37 || k == 42 || k == 46 || k == 49 || k == 51 || k == 53 || k == 55 || k == 59 || k == 82 || k == 84 || k == 85)     
      plot(acu_x2(k), acu_y2(k), 'bx', 'Linewidth', 5);
  end
%   if (k == 333 || k == 337 || k == 341 || k == 345 || k == 349 || k == 353 || k == 357 || k == 361 || k == 364 || k == 366 || k == 371 || k == 376 || k == 381 || k == 386 || k == 392 || k == 397 || k == 402 || k == 407 || k == 413 || k == 419 || k == 424 || k == 429 || k == 434 || k == 439 || k == 443)
%       plot(acu_x2(k), acu_y2(k), 'yx', 'Linewidth', 5);
%   end
%   if (k == 585)
%       plot(acu_x2(k), acu_y2(k), 'gx', 'Linewidth', 5);
%   end
%   
%     if (k == 5 || k == 26 || k == 49 || k == 86 || k == 130 || k == 214 || k == 285 || k == 322 || k == 359 || k == 383 || k == 411 || k == 462 || k == 501 || k == 541 || k == 578 || k == 626)
%       y1 = text(1, 4,'UAV_1 REFUEL','Color','red','FontSize',10);
%       caption1 = sprintf('stopped at %dth minute for refuel', k);
%       y2 = text(1, 3.5, caption1,'Color','red','FontSize',10);
%     end
  xlim([min(x(:)), max(x(:))]);
  ylim([min(y(:)), max(y(:))]);
  writeVideo(mov,getframe);
%     if (k == 5 || k == 26 || k == 49 || k == 86 || k == 130 || k == 214 || k == 285 || k == 322 || k == 359 || k == 383 || k == 411 || k == 462 || k == 501 || k == 541 || k == 578 || k == 626)
%         pause(0.1*speed_factor)
%     else
  pause(0.1*speed_factor)
%     end
  h = [uav_1, ugv];
  delete(h)
  del = [x0, z0, d];
  delete(del)
  

%   if (exist('y1','var')), delete(y1); end
  if (exist('y2','var')), delete(y2); end
  if (exist('time','var')), delete(time); end

end
%% Close the movie
close(mov);