"""This module plots the potential locations at which the UGV can make its stop such that UAV can visit the
mission points that are within the radial coverage of that stop so that UAV can visit those mission points and
come back to the UGV for recharging and fly off to perform its rest of the missions.

The radial coverage is plotted as a circle around a particular UGV stop"""

import math
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_excel('ARL corridor limited data points.xlsx', engine='openpyxl')

fig = plt.figure()
ax = fig.add_subplot(111)

x = []
y = []
# circle_x = 0
# circle_y = 0
circle_x1 = 2.77*5280
circle_y1 = 7.57*5280
circle_x2 = 0.6*5280
circle_y2 = 8.07*5280
circle_x3 = 8.75*5280
circle_y3 = 2.73*5280
# circle_x4 = 6.22168675*5280
# circle_y4 = 5.24939759*5280
# circle = plt.Circle((circle_x, circle_y), 24000, fill=True, alpha=0.4)  # speed = d/t => d = s x t => d = (33 x 1500)/2. Dividing by 2 to get radius.
# circle1 = plt.Circle((circle_x1, circle_y1), 24000, fill=True, alpha=0.4)
# circle2 = plt.Circle((circle_x2, circle_y2), 24000, fill=True, alpha=0.4)
# circle3 = plt.Circle((circle_x3, circle_y3), 24000, fill=True, alpha=0.4)
# circle4 = plt.Circle((circle_x4, circle_y4), 23622, fill=True, alpha=0.4)
# ax.add_artist(circle)
# ax.add_artist(circle1)
# ax.add_artist(circle2)
# ax.add_artist(circle3)
# ax.add_artist(circle4)
ax.set_aspect('equal', adjustable='box')

for i in range(len(df)):
    x.append(df['x (miles)'][i]*5280)
    y.append(df['y (miles)'][i]*5280)

for j in range(len(y)):
    if 0 <= j <= 9:
        ax.plot(x[j], y[j], 'rx', markersize=10)
    if 14 < j <= 21 or j == 11 or j == 12 or j == 13:
        ax.plot(x[j], y[j], 'bx', markersize=10)
    # if 22 < j <= 36:
    #     ax.plot(x[j], y[j], 'rx', markersize=10)
    if 31 <= j <= len(y):
        ax.plot(x[j], y[j], 'bx', markersize=10)
    if j == 10 or j == 14:
        ax.plot(x[j], y[j], 'bx', markersize=10)
    if j == 10 or j == 14:
        ax.plot(x[j], y[j], 'k.', markersize=10)
    elif j == 31:
        ax.plot(x[j], y[j], 'k.', markersize=10)
    elif 22 <= j < 31:
        ax.plot(x[j], y[j], 'kx', markersize=10)

plt.savefig('loc_initial_hyperparam.pdf')
plt.show()

