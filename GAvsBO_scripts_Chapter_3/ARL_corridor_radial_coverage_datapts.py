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
circle_x1 = 3.1*5280
circle_y1 = 7.24*5280
circle_x2 = 0.6*5280
circle_y2 = 8.07*5280
circle_x3 = 10.54*5280
circle_y3 = 0.9*5280
# circle_x4 = 6.22168675*5280
# circle_y4 = 5.24939759*5280
# circle = plt.Circle((circle_x, circle_y), 24000, fill=True, alpha=0.4)  # speed = d/t => d = s x t => d = (33 x 1500)/2. Dividing by 2 to get radius.
# circle1 = plt.Circle((circle_x1, circle_y1), 24000, fill=True, alpha=0.4)
# circle2 = plt.Circle((circle_x2, circle_y2), 24000, fill=True, alpha=0.4)
# circle3 = plt.Circle((circle_x3, circle_y3), 24000, fill=True, alpha=0.4)
# # circle4 = plt.Circle((circle_x4, circle_y4), 23622, fill=True, alpha=0.4)
# # ax.add_artist(circle)
# ax.add_artist(circle1)
# ax.add_artist(circle2)
# ax.add_artist(circle3)
# ax.add_artist(circle4)
ax.set_aspect('equal', adjustable='box')

for i in range(len(df)):
    x.append(df['x (miles)'][i]*5280)
    y.append(df['y (miles)'][i]*5280)

for j in range(len(y)):
    if j == 11 or j == 14:
        ax.plot(x[j], y[j], 'k.', markersize=20)
    elif j == 22:
        ax.plot(x[j], y[j], 'k.', markersize=20)
    elif 12 <= j <= 48:
        ax.plot(x[j], y[j], 'b.', markersize=10)
    else:
        ax.plot(x[j], y[j], 'r.', markersize=10)

# plt.savefig('problem_description_without_radial_coverage.pdf')
plt.show()

