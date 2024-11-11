"""This module plots the potential locations at which the UGV can make its stop such that UAV can visit the
mission points that are within the radial coverage of that stop so that UAV can visit those mission points and
come back to the UGV for recharging and fly off to perform its rest of the missions.

The radial coverage is plotted as a circle around a particular UGV stop"""

import math
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('Scenario dataset/Scenario 1 data points.csv')

fig = plt.figure()
ax = fig.add_subplot(111)

x = []
y = []
circle_x = 0
circle_y = 0
circle_x1 = 7.98*5280
circle_y1 = 3.78*5280
# circle_x2 = 7.98*5280
# circle_y2 = 3.78*5280
circle_x3 = 5.16*5280
circle_y3 = 5.24*5280
# circle_x4 = 5.52*5280
# circle_y4 = 6.66*5280
circle = plt.Circle((circle_x, circle_y), 23622, fill=True, alpha=0.4)  # speed = d/t => d = s x t => d = (33 x 1500)/2. Dividing by 2 to get radius.
circle1 = plt.Circle((circle_x1, circle_y1), 23622, fill=True, alpha=0.4)
# circle2 = plt.Circle((circle_x2, circle_y2), 23622, fill=True, alpha=0.4)
circle3 = plt.Circle((circle_x3, circle_y3), 23622, fill=True, alpha=0.4)
# circle4 = plt.Circle((circle_x4, circle_y4), 23622, fill=True, alpha=0.4)
ax.add_artist(circle)
ax.add_artist(circle1)
# ax.add_artist(circle2)
ax.add_artist(circle3)
# ax.add_artist(circle4)
ax.set_aspect('equal', adjustable='box')

for i in range(len(df)):
    x.append(df['x (miles)'][i]*5280)
    y.append(df['y (miles)'][i]*5280)

for j in range(len(y)):
    if j == 31:
        ax.plot(x[j], y[j], 'k.', markersize=10)
    elif j == 0:
        ax.plot(x[j], y[j], 'k.', markersize=10)
    else:
        ax.plot(x[j], y[j], 'kx', markersize=10)

plt.savefig('Scenario 1 with radial coverage.pdf')
plt.show()

