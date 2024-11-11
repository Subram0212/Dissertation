clc
close all
recharge_time = 14;
table = readtable('data_to_interpolate_UIC_UMD_depotbstrt_2UAVs1UGV_part2.xlsx');
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

m = 1;
while m <= (length(x_prime)-recharge_time)
    if ((x_prime(m)==0.6 && y_prime(m)==8.07) && m ~= 1 && (m == 22 || m == 39 || m == 54 || m == 70 || m == 90))
        for n=m:m+recharge_time
            x_prime(n) = 0.6;
            y_prime(n) = 8.07;
        end
        m = m + 13;
    elseif ((x_prime(m)==3.1 && y_prime(m)==7.24) && m ~= 1 && m == 59)
        for n=m:m+5
            x_prime(n) = 3.1;
            y_prime(n) = 7.24;
        end
        m = m + 15;
%     elseif ((x_prime(m)==3.1 && y_prime(m)==7.24) && m ~= 1 && m == 54)
%         for n=m:m+4
%             x_prime(n) = 3.1;
%             y_prime(n) = 7.24;
%         end
%         m = m + 7;
%     elseif ((x_prime(m)==3.1 && y_prime(m)==7.24) && m ~= 1 && m == 70)
%         for n=m:m+4
%             x_prime(n) = 3.1;
%             y_prime(n) = 7.24;
%         end
%         m = m + 5;
%     elseif ((x_prime(m)==3.1 && y_prime(m)==7.24) && m ~= 1 && m == 7)
%         for n=m:m+8
%             x_prime(n) = 3.1;
%             y_prime(n) = 7.24;
%         end
%         m = m + 8;
%     elseif ((x_prime(m)==10.54 && y_prime(m)==0.9) && m ~= 1)
%         for n=m:m+15
%             x_prime(n) = 10.54;
%             y_prime(n) = 0.9;
%         end
%         m = m + 15;
%     elseif ((x_prime(m)==8.04 && y_prime(m)==3.28) && m ~= 1)
%         for n=m:m+recharge_time
%             x_prime(n) = 8.04;
%             y_prime(n) = 3.28;
%         end
%         m = m + recharge_time;
%     elseif ((x_prime(m)==8.91 && y_prime(m)==4.54) && m ~= 1)
%         for n=m:m+14
%             x_prime(n) = 8.91;
%             y_prime(n) = 4.54;
%         end
%         m = m + recharge_time;
    end
    m = m + 1;
end

xq = [];
ugv = [];
k = find(x_prime);
u = find(x1_prime);

for i=2:length(k)
    xq(i-1) = k(i) - k(i-1);
end
for p=2:length(u)
    ugv(p-1) = u(p) - u(p-1);
end
interpolate_x = [];
interpolate_y = [];
interpolate_x1 = [];
interpolate_y1 = [];
for j=2:length(k)
    interpolate_x(j-1) = (x_prime(k(j)) - x_prime(k(j-1)))/xq(j-1);
    interpolate_y(j-1) = (y_prime(k(j)) - y_prime(k(j-1)))/xq(j-1);
end
interpolate_x(length(k)) = 0;
interpolate_y(length(k)) = 0;
for q=2:length(u)
    interpolate_x1(q-1) = (x1_prime(u(q)) - x1_prime(u(q-1)))/ugv(q-1);
    interpolate_y1(q-1) = (y1_prime(u(q)) - y1_prime(u(q-1)))/ugv(q-1);
end
interpolate_x1(length(u)) = 0;
interpolate_y1(length(u)) = 0;

int_value_x = 0;
int_value_y = 0;
int_value_x1 = 0;
int_value_y1 = 0;
for l=1:(length(x_prime)-1)
    if (ismember(l, k))
        index = find(k==l);
        int_value_x = interpolate_x(index);
        int_value_y = interpolate_y(index);
    else
        x_prime(l) = x_prime(l-1) + int_value_x;
        y_prime(l) = y_prime(l-1) + int_value_y;
    end
end

for r=1:(length(x1_prime)-1)
    if (ismember(r, u))
        index = find(u==r);
        int_value_x1 = interpolate_x1(index);
        int_value_y1 = interpolate_y1(index);
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

writetable(data_to_animate, 'Interpolated_data_UIC_UMD_planner_results_part2.xlsx');
% elseif (ismember(l, u))
%         index = find(u==l);
%         int_value_x1 = interpolate_x1(index+1);
%         int_value_y1 = interpolate_y1(index+1);
% x1_prime(l) = x1_prime(l-1) + int_value_x1;
% y1_prime(l) = y1_prime(l-1) + int_value_y1;

% xlswrite('Interpolated_data.xlsx', original_x, 'B');
% vq1 = interp1(x_prime, y_prime, xq);