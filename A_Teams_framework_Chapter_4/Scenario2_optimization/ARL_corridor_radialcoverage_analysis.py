import math
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('Scenario 2 data points.csv')

fig = plt.figure()
ax = fig.add_subplot(111)

x = []
y = []
circle_x = 0*5280
circle_y = 0*5280
circle_x1 = 5.35*5280
circle_y1 = 6.1*5280
circle_x2 = 5.56*5280
circle_y2 = 5.89*5280
# circle_x3 = 8.660254038*5280
# circle_y3 = 5*5280
# circle_x4 = 6.22168675*5280
# circle_y4 = 5.24939759*5280
circle = plt.Circle((circle_x, circle_y), 23622, fill=True, alpha=0.4) # speed = d/t => d = s x t => d = (33 x 1500)/2. Dividing by 2 to get radius.
circle1 = plt.Circle((circle_x1, circle_y1), 23622, fill=True, alpha=0.4)
circle2 = plt.Circle((circle_x2, circle_y2), 23622, fill=True, alpha=0.4)
# circle3 = plt.Circle((circle_x3, circle_y3), 23622, fill=True, alpha=0.4)
# circle4 = plt.Circle((circle_x4, circle_y4), 23622, fill=True, alpha=0.4)
ax.add_artist(circle)
ax.add_artist(circle1)
ax.add_artist(circle2)
# ax.add_artist(circle3)
# ax.add_artist(circle4)
ax.set_aspect('equal', adjustable='box')

for i in range(len(df)):
    x.append(df['x (miles)'][i]*5280)
    y.append(df['y (miles)'][i]*5280)

for j in range(len(y)):
    if j == 17 or j == 33:
        ax.plot(x[j], y[j], 'kx', markersize=10)
    elif j == 16 or j == 32:
        ax.plot(x[j], y[j], 'k.', markersize=10)
    # elif j == 24 or j == 26 or j == 28 or j == 30 or j == 32 or j == 34 or j == 36 or j == 38 or j == 40 or j == 42 \
    #         or j == 44:
    #     ax.plot(x[j], y[j], 'k.', markersize=10)
    #     ax.plot(x[j], y[j], 'kx', markersize=10)
    else:
        ax.plot(x[j], y[j], 'kx', markersize=10)

# plt.xlim(-1, 15)
# plt.ylim(-1, 15)
plt.savefig('Scenario 2 with radial coverage.pdf')
plt.show()

