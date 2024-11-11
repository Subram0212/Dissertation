"""This module plots the potential locations at which the UGV can make its stop such that UAV can visit the
mission points that are within the radial coverage of that stop so that UAV can visit those mission points and
come back to the UGV for recharging and fly off to perform its rest of the missions.

The radial coverage is plotted as a circle around a particular UGV stop"""

import math

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('Mission locations hardware - From Mocap.csv')

fig = plt.figure()
ax = fig.add_subplot(111)
# img = mpimg.imread('figure.png')
# ax.imshow(img, aspect='auto', extent=[-0.75, 18, -0.75, 18])

x = []
y = []
# circle_x = 0
# circle_y = 0
circle_x1 = 0.095*100
circle_y1 = 1.761*100
circle_x2 = 0.832*100
circle_y2 = 1.533*100
circle_x3 = 1.707*100
circle_y3 = 0.879*100
# circle_x4 = 5.52*5280
# circle_y4 = 6.66*5280
# circle = plt.Circle((circle_x, circle_y), 23622, fill=True, alpha=0.4)  # speed = d/t => d = s x t => d = (33 x 1500)/2. Dividing by 2 to get radius.
circle1 = plt.Circle((circle_x1, circle_y1), 98, fill=True, alpha=0.4)
circle2 = plt.Circle((circle_x2, circle_y2), 100, fill=True, alpha=0.4)
circle3 = plt.Circle((circle_x3, circle_y3), 98, fill=True, alpha=0.4)
# circle4 = plt.Circle((circle_x4, circle_y4), 23622, fill=True, alpha=0.4)
# ax.add_artist(circle)
# ax.add_artist(circle1)
# ax.add_artist(circle2)
# ax.add_artist(circle3)
# ax.add_artist(circle4)
ax.set_aspect('equal', adjustable='box')
# plt.xlim([-0.25, 3])
# plt.ylim([-0.25, 3])

for i in range(len(df)):
    x.append(df['x (m)'][i]*100)
    y.append(df['y (m)'][i]*100)

for j in range(len(y)):
    # if j == 27:
    #     ax.plot(x[j], y[j], 'k.', markersize=10)
    # elif j == 39:
    #     ax.plot(x[j], y[j], 'k.', markersize=10)
    # else:
    ax.plot(x[j], y[j], 'kx', markersize=10)

plt.savefig('Scenario for hardware experiment coverage msc.pdf')
plt.show()

