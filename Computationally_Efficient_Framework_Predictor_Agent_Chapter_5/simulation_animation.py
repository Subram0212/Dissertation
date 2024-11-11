import copy

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import numpy as np
import matplotlib.patches as patches
from matplotlib.lines import Line2D
plt.rcParams.update({'font.size': 13})


def add_circle_to_x(ax, x, y, drone_color, size=0.05, circle_radius=0.035, alpha=0.5):
    # Create the 'x' mark as two diagonal lines
    x_coords = [x - size, x + size]
    y_coords = [y - size, y + size]
    line1 = ax.plot(x_coords, y_coords, color=drone_color, linewidth=2, alpha=alpha)[0]
    x_coords = [x - size, x + size]
    y_coords = [y + size, y - size]
    line2 = ax.plot(x_coords, y_coords, color=drone_color, linewidth=2, alpha=alpha)[0]

    # Add circles at all four corners of the 'x' mark
    circle1 = patches.Circle((x - size, y - size), circle_radius, edgecolor=drone_color, facecolor='none', alpha=alpha, linewidth=2)
    circle2 = patches.Circle((x + size, y - size), circle_radius, edgecolor=drone_color, facecolor='none', alpha=alpha, linewidth=2)
    circle3 = patches.Circle((x - size, y + size), circle_radius, edgecolor=drone_color, facecolor='none', alpha=alpha, linewidth=2)
    circle4 = patches.Circle((x + size, y + size), circle_radius, edgecolor=drone_color, facecolor='none', alpha=alpha, linewidth=2)
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.add_patch(circle3)
    ax.add_patch(circle4)

    return line1, line2, circle1, circle2, circle3, circle4


# Get the data
sim_data_uav_ugv = pd.read_excel('Interpolated_data_hardware_simulation_CASE_PS_Final_3_13_24.xlsx')
# sim_data_uav_ugv = pd.read_excel('Interpolated_data_hardware_simulation_CASE_PS_Offline.xlsx')
sim_data_uav_ugv_arr = sim_data_uav_ugv.to_numpy()
# mission_locations_simulation = pd.read_csv('Mission locations hardware - From Mocap.csv')
# mocap_data = pd.read_csv('Persistent surveillance mocap realtime data final - Improved case for CASE 3_13_24.csv')
mission_locations_hardware = pd.read_csv('Hardware experimental scaled up case study scenario in miles.csv')
# mocap_data_x_uav = mocap_data['UAV_X']
# mocap_data_y_uav = mocap_data['UAV_Y']
# mocap_data_x_ugv = mocap_data['UGV_X']
# mocap_data_y_ugv = mocap_data['UGV_Y']
mission_locs_x = mission_locations_hardware['X']
mission_locs_y = mission_locations_hardware['Y']
mission_locs_x = [round(i*0.22526, 2) for i in mission_locs_x if i not in [10.55, 11]]
mission_locs_y = [round(i*0.22526, 2) for i in mission_locs_y if i not in [0.45, 1.25]]
mission_locs_x_temp = copy.deepcopy(mission_locs_x)
mission_locs_y_temp = copy.deepcopy(mission_locs_y)
index_gap = 120
indices = np.linspace(0, len(sim_data_uav_ugv_arr) - 1, len(sim_data_uav_ugv_arr) * index_gap + 1)

# Use numpy's interp function to get the interpolated values
interpolated_data = np.zeros((len(indices), sim_data_uav_ugv_arr.shape[1]))

# Perform interpolation for each column
for col in range(sim_data_uav_ugv_arr.shape[1]):
    interpolated_data[:, col] = np.interp(indices, range(len(sim_data_uav_ugv_arr)), sim_data_uav_ugv_arr[:, col])

# desired_length = len(mocap_data_x_uav)
#
# if len(interpolated_data) < len(mocap_data_x_uav):
#     last_row = interpolated_data[-1, :]
#     rows_to_fill = mocap_data_x_uav.shape[0] - interpolated_data.shape[0]
#     interpolated_data = np.vstack([interpolated_data, np.tile(last_row, (rows_to_fill, 1))])

sim_data_x_uav = sim_data_uav_ugv['x']
sim_data_y_uav = sim_data_uav_ugv['y']
sim_data_x_ugv = sim_data_uav_ugv['x_1']
sim_data_y_ugv = sim_data_uav_ugv['y_1']

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([0, 2.75])
ax.set_ylim([0, 2.75])

# Add labels and title
ax.set_xlabel('X (cm)')
ax.set_ylabel('Y (cm)')
# ax.set_title('Animated Plot')
# Iterate over each frame and update the plot
count = 1
stamp = 0
condn_exec = 1
txt = ax.text(0.25, 0.25, '', visible=False)
scn_map, = ax.plot(mission_locs_x, mission_locs_y, 'ko', markersize=6, linewidth=0.5, label='Task points')
circle_proxy = add_circle_to_x(ax, 1.65, 2.125, 'red', size=0.05, circle_radius=0.035, alpha=1)
plt.text(1.875, 2.075, 'UAV')
# ax.legend([circle_proxy], ['UAV'])
plt.text(0.15, 1.95, 'Depot')
dynamic_node_flag = 0
border_color = (0.5, 0.5, 0.5, 0.5)
rect = patches.Rectangle((1.5, 2), 1.15, 0.65, linewidth=1, edgecolor=border_color, facecolor='none')
# Add the rectangle to the Axes
ax.add_patch(rect)
for i in range(0, len(sim_data_x_uav)):
    if i > 20:
        txt.set_visible(False)  # Hide previous text to avoid overlap
        txt.remove()  # Remove the previous text object
        # Add new text for the current condition
        txt = ax.text(0.125, 0.125, 'Elapsed T = ' + str(round(i/1.01)-20) + ' sec', color='green', fontweight='bold', fontsize=10)
        txt.set_visible(True)  # Make sure the new text is visible
    # Case 1 of Dynamic scenarios
    if 13500 <= i <= 39000:
        dynamic_locs = [(3*0.22526, 4.5*0.22526), (9.82*0.22526, 3.5*0.22526)]
        dynamic_locs_x = [i[0] for i in dynamic_locs]
        dynamic_locs_y = [i[1] for i in dynamic_locs]
        scn_map2, = ax.plot(dynamic_locs_x, dynamic_locs_y, 'kx', markersize=15)
    elif i >= 39000:
        dynamic_locs = [(1.45*0.22526, 10.5*0.22526), (5.25*0.22526, 8.5*0.22526)]
        dynamic_locs_x = [i[0] for i in dynamic_locs]
        dynamic_locs_y = [i[1] for i in dynamic_locs]
        scn_map2, = ax.plot(dynamic_locs_x, dynamic_locs_y, 'kx', markersize=15)
    else:
        scn_map2, = ax.plot(0.19, 1.81, 'kx-', markersize=15)
    # Case 2 of Dynamic scenarios
    if 7000//120 <= i <= 37200//120:
        dynamic_locs = [(6.5*0.22526, 1.5*0.22526), (7.2*0.22526, 6*0.22526)]
        dynamic_locs_x = [i[0] for i in dynamic_locs]
        dynamic_locs_y = [i[1] for i in dynamic_locs]
        if dynamic_node_flag == 1:
            scn_map2, = ax.plot(7.2*0.22526, 6*0.22526, 'ro', markeredgewidth=2, markersize=5)
            scn_map5, = ax.plot(6.5*0.22526, 1.5*0.22526, 'ko', markeredgewidth=2, markersize=5)
        elif dynamic_node_flag == 2:
            scn_map6, = ax.plot(7.2*0.22526, 6*0.22526, 'ro', markeredgewidth=2, markersize=5)
            scn_map2, = ax.plot(6.5*0.22526, 1.5*0.22526, 'ro', markeredgewidth=2, markersize=5)
        else:
            scn_map2, = ax.plot(dynamic_locs_x, dynamic_locs_y, 'ko', markeredgewidth=2, markersize=5)
    elif i >= 38500//120:
        scn_map5, = ax.plot(6.5*0.22526, 1.5*0.22526, 'wo', markeredgewidth=2, markersize=7)
        scn_map6, = ax.plot(7.2*0.22526, 6*0.22526, 'wo', markeredgewidth=2, markersize=7)
        dynamic_locs = [(1.5*0.22526, 10.5*0.22526), (5.25*0.22526, 8.5*0.22526)]
        dynamic_locs_x = [i[0] for i in dynamic_locs]
        dynamic_locs_y = [i[1] for i in dynamic_locs]
        if dynamic_node_flag == 3:
            scn_map2, = ax.plot(1.5*0.22526, 10.5*0.22526, 'ro', markeredgewidth=2, markersize=5)
            scn_map7, = ax.plot(5.25*0.22526, 8.5*0.22526, 'ko', markeredgewidth=2, markersize=5)
        elif dynamic_node_flag == 4:
            scn_map8, = ax.plot(1.5*0.22526, 10.5*0.22526, 'ro', markeredgewidth=2, markersize=5)
            scn_map2, = ax.plot(5.25*0.22526, 8.5*0.22526, 'ro', markeredgewidth=2, markersize=5)
        else:
            scn_map2, = ax.plot(dynamic_locs_x, dynamic_locs_y, 'ko', markeredgewidth=2, markersize=5)
    else:
        scn_map2, = ax.plot(0.19, 1.81, 'ko', markersize=10)
    # Clear the previous plot
    # ax.set_xlim([0, 3])
    # ax.set_ylim([0, 3])
    # ax.clear()
    lines1 = []
    patches_list1 = []
    lines2 = []
    patches_list2 = []
    # print(f"Dynamic flag: {dynamic_node_flag}")

    # Plot the data up to the current frame
    # plot_fig = ax.scatter(mocap_data_x_uav[i], -1*mocap_data_y_uav[i] + 1.5, marker='x', s=75, color='r', cmap='dark', label='UAV Hardware')
    # line1, line2, *circles = add_circle_to_x(ax, mocap_data_x_uav[i], mocap_data_y_uav[i], 'red', alpha=1)
    # lines1.extend([line1, line2])
    # patches_list1.extend(circles)
    # plot_fig2 = ax.scatter(mocap_data_x_ugv[i], mocap_data_y_ugv[i], marker='s', s=100, color='r', cmap='light', label='Hardware')
    # plot_fig3 = ax.scatter(sim_data_x_uav[i]*0.22526, sim_data_y_uav[i]*0.22526, marker='x', s=75, color='b', cmap='dark', label='UAV Simulation')
    line3, line4, *circles2 = add_circle_to_x(ax, sim_data_x_uav[i]*0.22526, sim_data_y_uav[i]*0.22526, 'red', alpha=1)
    lines2.extend([line3, line4])
    patches_list2.extend(circles2)
    plot_fig4 = ax.scatter(sim_data_x_ugv[i]*0.22526, sim_data_y_ugv[i]*0.22526, marker='s', s=300, color='b', cmap='light', label='UGV', alpha=1)
    rounded_sim_data_x_uav = round(sim_data_x_uav[i]*0.22526, 2)
    rounded_sim_data_y_uav = round(sim_data_y_uav[i]*0.22526, 2)
    rounded_sim_data_x_ugv = round(sim_data_x_ugv[i]*0.22526, 2)
    rounded_sim_data_y_ugv = round(sim_data_y_ugv[i]*0.22526, 2)
    if (rounded_sim_data_x_uav in mission_locs_x_temp and rounded_sim_data_y_uav in mission_locs_y_temp and i != 61) or (rounded_sim_data_x_uav in [round(6.5*0.22526, 2), round(7.2*0.22526, 2), round(1.5*0.22526, 2), round(5.25*0.22526, 2)] and rounded_sim_data_y_uav in [round(1.5*0.22526, 2), round(6*0.22526, 2), round(10.5*0.22526, 2), round(8.5*0.22526, 2)]):
        plot_tmp1_uav, = ax.plot(sim_data_x_uav[i]*0.22526, sim_data_y_uav[i]*0.22526, 'ro', markersize=5, markeredgewidth=2)
        if (rounded_sim_data_x_uav in [round(6.5*0.22526, 2), round(7.2*0.22526, 2), round(1.5*0.22526, 2), round(5.25*0.22526, 2)] and rounded_sim_data_y_uav in [round(1.5*0.22526, 2), round(6*0.22526, 2), round(10.5*0.22526, 2), round(8.5*0.22526, 2)]):
            dynamic_node_flag += 1

    if (rounded_sim_data_x_ugv in mission_locs_x_temp and rounded_sim_data_y_ugv in mission_locs_y_temp):
        plot_tmp2_ugv, = ax.plot(sim_data_x_ugv[i]*0.22526, sim_data_y_ugv[i]*0.22526, 'bo', markersize=5, markeredgewidth=2)

    legend = plt.legend(loc='upper right', frameon=False)

    # Pause for a brief time to create animation effect
    plt.pause(0.05)
    # plot_fig.remove()
    for line in lines1:
        line.remove()
    for patch in patches_list1:
        patch.remove()
    for line in lines2:
        line.remove()
    for patch in patches_list2:
        patch.remove()
    # plot_fig2.remove()
    # plot_fig3.remove()
    plot_fig4.remove()
    # scn_map.remove()
    # scn_map2.remove()
    # scn_map3.remove()

# Display the animation
plt.show()

