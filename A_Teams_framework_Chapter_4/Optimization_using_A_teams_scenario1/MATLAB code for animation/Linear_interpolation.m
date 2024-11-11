clc
close all
recharge_time = 14;
table = readtable('data_to_interpolate_GA_LS_algorithm_scenario1_depotA start.xlsx');
x = table.x;
y = table.y;
time = table.time;
x1 = table.x1;
y1 = table.y1;

x_prime = transpose(x);
y_prime = transpose(y);
time_prime = transpose(time);
x1_prime = transpose(x1);
y1_prime = transpose(y1);

% x_vec = [];
% y_vec = [];
% count = 2;

% m = 2;
% while m <= (length(x_prime)-recharge_time)
%     if ((x_prime(m)==0 && y_prime(m)==0) && m ~= 1 && m == 140)
%         for n=m:m+recharge_time
%             x_prime(n) = 0;
%             y_prime(n) = 0;
%         end
%         m = m + recharge_time;
%     elseif ((x_prime(m)==7.37 && y_prime(m)==3.27) && m ~= 1)
%         for n=m:m+recharge_time
%             x_prime(n) = 7.37;
%             y_prime(n) = 3.27;
%         end
%         m = m + 14;
%     elseif ((x_prime(m)==5.16 && y_prime(m)==5.24) && m ~= 1)
%         for n=m:m+recharge_time
%             x_prime(n) = 5.16;
%             y_prime(n) = 5.24;
%         end
%         m = m + 15;
% %     elseif ((x_prime(m)==8.04 && y_prime(m)==3.28) && m ~= 1)
% %         for n=m:m+recharge_time
% %             x_prime(n) = 8.04;
% %             y_prime(n) = 3.28;
% %         end
% %         m = m + recharge_time;
%     elseif ((x_prime(m)==8.91 && y_prime(m)==4.54) && m ~= 1)
%         for n=m:m+14
%             x_prime(n) = 8.91;
%             y_prime(n) = 4.54;
%         end
%         m = m + recharge_time;
%     end
%     m = m + 1;
% end

xq = [];
ugv = [];
k = find(x_prime);
u = find(x1_prime);

for i=2:length(k)
    xq(i) = k(i) - k(i-1);
end
for p=2:length(u)
    ugv(p) = u(p) - u(p-1);
end
interpolate_x = [];
interpolate_y = [];
interpolate_x1 = [];
interpolate_y1 = [];
for j=2:length(k)
    interpolate_x(j) = (x_prime(k(j)) - x_prime(k(j-1)))/xq(j);
    interpolate_y(j) = (y_prime(k(j)) - y_prime(k(j-1)))/xq(j);
end
for q=2:length(u)
    interpolate_x1(q) = (x1_prime(u(q)) - x1_prime(u(q-1)))/ugv(q);
    interpolate_y1(q) = (y1_prime(u(q)) - y1_prime(u(q-1)))/ugv(q);
end

int_value_x = 0;
int_value_y = 0;
int_value_x1 = 0;
int_value_y1 = 0;
for l=2:(length(x_prime)-1)
    if (ismember(l, k) && l ~= 1)
        index = find(k==l);
        int_value_x = interpolate_x(index+1);
        int_value_y = interpolate_y(index+1);
    else
        x_prime(l) = x_prime(l-1) + int_value_x;
        y_prime(l) = y_prime(l-1) + int_value_y;
    end
end

for r=1:(length(x1_prime)-1)
    if (ismember(r, u) && l ~= 1)
        index = find(u==r);
        int_value_x1 = interpolate_x1(index+1);
        int_value_y1 = interpolate_y1(index+1);
    else
        x1_prime(r) = x1_prime(r-1) + int_value_x1;
        y1_prime(r) = y1_prime(r-1) + int_value_y1;
    end
end

original_x = transpose(x_prime);
original_y = transpose(y_prime);
original_x1 = transpose(x1_prime);
original_y1 = transpose(y1_prime);
array_data_to_animate = [original_x original_y original_x1 original_y1];
data_to_animate = array2table(array_data_to_animate);

writetable(data_to_animate, 'Interpolated_data_GA_LS_scenario1.xlsx');
% elseif (ismember(l, u))
%         index = find(u==l);
%         int_value_x1 = interpolate_x1(index+1);
%         int_value_y1 = interpolate_y1(index+1);
% x1_prime(l) = x1_prime(l-1) + int_value_x1;
% y1_prime(l) = y1_prime(l-1) + int_value_y1;

% xlswrite('Interpolated_data.xlsx', original_x, 'B');
% vq1 = interp1(x_prime, y_prime, xq);