import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import numpy as np
import matplotlib.patches as patches
import cv2
import os


def round_and_strip(number, decimal_places):
    # Round the number to the specified decimal places
    rounded_str = "{:.{}f}".format(number, decimal_places)

    # Remove trailing zeros and the decimal point if the result is an integer
    rounded_str = rounded_str.rstrip('0').rstrip('.')

    return rounded_str


def add_circle_to_x(ax, x, y, drone_color, size=0.15, circle_radius=0.1):
    # Create the 'x' mark as two diagonal lines
    x_coords = [x - size, x + size]
    y_coords = [y - size, y + size]
    line1 = ax.plot(x_coords, y_coords, color=drone_color, linewidth=1.75)[0]
    x_coords = [x - size, x + size]
    y_coords = [y + size, y - size]
    line2 = ax.plot(x_coords, y_coords, color=drone_color, linewidth=1.75)[0]

    # Add circles at all four corners of the 'x' mark
    circle1 = patches.Circle((x - size, y - size), circle_radius, edgecolor=drone_color, facecolor='none')
    circle2 = patches.Circle((x + size, y - size), circle_radius, edgecolor=drone_color, facecolor='none')
    circle3 = patches.Circle((x - size, y + size), circle_radius, edgecolor=drone_color, facecolor='none')
    circle4 = patches.Circle((x + size, y + size), circle_radius, edgecolor=drone_color, facecolor='none')
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.add_patch(circle3)
    ax.add_patch(circle4)
    return line1, line2, circle1, circle2, circle3, circle4


# Get the data
# sim_data_uav_ugv = pd.read_excel('Interpolated_data_scaledup_sim_for_hardware_for_experimental_comparison_v4.xlsx')
# sim_data_uav_ugv_arr = sim_data_uav_ugv.to_numpy()
# mission_locations_simulation = pd.read_csv('Mission locations hardware - From Mocap.csv')
animation_data = pd.read_excel('Interpolated_data_scenario2_ensemble.xlsx')
mission_locations_hardware = pd.read_csv('Case_Study_scenario6.csv')
mocap_data_x_uav = animation_data['x']
mocap_data_y_uav = animation_data['y']
mocap_data_x_ugv = animation_data['x_1']
mocap_data_y_ugv = animation_data['y_1']
# mocap_data_x_ugv = round(mocap_data_x_ugv, 10)
# mocap_data_y_ugv = round(mocap_data_y_ugv, 9)
mission_locs_x = mission_locations_hardware['X']
mission_locs_y = mission_locations_hardware['Y']
mission_locs_x_rnd = round(mission_locs_x, 2)
mission_locs_y_rnd = round(mission_locs_y, 2)
mission_locs_x = mission_locs_x.values.tolist()
mission_locs_y = mission_locs_y.values.tolist()
mission_locs_x_rnd = mission_locs_x_rnd.values.tolist()
mission_locs_y_rnd = mission_locs_y_rnd.values.tolist()

index_gap = 60
# indices = np.linspace(0, len(sim_data_uav_ugv_arr) - 1, len(sim_data_uav_ugv_arr) * index_gap + 1)

# Use numpy's interp function to get the interpolated values
# interpolated_data = np.zeros((len(indices), sim_data_uav_ugv_arr.shape[1]))

# Perform interpolation for each column
# for col in range(sim_data_uav_ugv_arr.shape[1]):
#     interpolated_data[:, col] = np.interp(indices, range(len(sim_data_uav_ugv_arr)), sim_data_uav_ugv_arr[:, col])

desired_length = len(mocap_data_x_uav)

# if len(interpolated_data) < len(mocap_data_x_uav):
#     last_row = interpolated_data[-1, :]
#     rows_to_fill = mocap_data_x_uav.shape[0] - interpolated_data.shape[0]
#     interpolated_data = np.vstack([interpolated_data, np.tile(last_row, (rows_to_fill, 1))])

# sim_data_x_uav = interpolated_data[:, 0]
# sim_data_y_uav = interpolated_data[:, 1]
# sim_data_x_ugv = interpolated_data[:, 2]
# sim_data_y_ugv = interpolated_data[:, 3]
frame_size = (1617,1080)
fig, ax = plt.subplots()
ax.set_xlim([0, 11])
ax.set_ylim([0, 12])

# Add labels and title
ax.set_xlabel('X (km)')
ax.set_ylabel('Y (km)')
ax.set_title('Animated Plot')
img = mpimg.imread('figure.png')
ax.imshow(img, aspect='auto', extent=[0, 11, 0, 12])
scn_map, = ax.plot(mission_locs_x, mission_locs_y, 'kx', markersize=5)
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Choose codec
# video_path = os.path.join('ScenarioARL_route_animation.mp4')
# # out_anim = cv2.VideoWriter(video_path, fourcc, 5, (3000,3000))
# out_anim = cv2.VideoWriter(video_path, fourcc, 5, frame_size)# (1920,1080))

# Iterate over each frame and update the plot
for i in range(0, len(mocap_data_x_uav)):
    txt = plt.text(5, 11, 'Wall time elapsed = '+str(i-11)+'minutes', color='black')
    # Clear the previous plot
    # ax.set_xlim([0, 3])
    # ax.set_ylim([0, 3])
    # ax.clear()
    lines1 = []
    patches_list1 = []
    lines2 = []
    patches_list2 = []

    # Plot the data up to the current frame
    # plot_fig = ax.scatter(mocap_data_x_uav[i], -1*mocap_data_y_uav[i] + 1.5, marker='x', s=75, color='r', cmap='dark', label='UAV Hardware')
    line1, line2, *circles = add_circle_to_x(ax, mocap_data_x_uav[i], mocap_data_y_uav[i], 'red')
    lines1.extend([line1, line2])
    patches_list1.extend(circles)
    plot_fig2 = ax.scatter(mocap_data_x_ugv[i], mocap_data_y_ugv[i], marker='s', s=100, color='b', cmap='light')
    # plot_fig3 = ax.scatter(sim_data_x_uav[i]*0.22526, sim_data_y_uav[i]*0.22526, marker='x', s=75, color='b', cmap='dark', label='UAV Simulation')
    if (mocap_data_x_uav[i] in mission_locs_x_rnd or mocap_data_y_uav[i] in mission_locs_y_rnd):
        plot_tmp1, = ax.plot(mocap_data_x_uav[i], mocap_data_y_uav[i], 'rx', markersize=5, markeredgewidth=2)
    if mocap_data_x_ugv[i] in mission_locs_x or mocap_data_y_ugv[i] in mission_locs_y:
        plot_tmp2, = ax.plot(mocap_data_x_ugv[i], mocap_data_y_ugv[i], 'bx', markersize=5, markeredgewidth=2)

    # legend = plt.legend(loc='upper right')

    # Pause for a brief time to create animation effect
    plt.pause(0.1)
    # plot_fig.remove()
    for line in lines1:
        line.remove()
    for patch in patches_list1:
        patch.remove()
    for line in lines2:
        line.remove()
    for patch in patches_list2:
        patch.remove()
    plot_fig2.remove()
    txt.remove()
    # plot_fig3.remove()
    # plot_fig4.remove()
    # figure = fig
    # figure.canvas.draw()
    # frame = np.array(figure.canvas.renderer.buffer_rgba()) # Convert the plot to a numpy array
    # frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR) # Convert the color format from RGBA to BGR
    # # frame = cv2.resize(frame, (3000, 3000))
    # frame = cv2.resize(frame, frame_size)# (1920,1080))
    # out_anim.write(frame)


# Display the animation
plt.show()
# out_anim.release()

