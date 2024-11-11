import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv('Case_Study_scenario3.csv')
x = data['X']
y = data['Y']

x = x.tolist()
y = y.tolist()

x = [i*1.61 for i in x]
y = [i*1.61 for i in y]

fig, ax = plt.subplots()

ax.plot(x, y, 'kx')
ax.set_aspect('equal')
ax.set_xlim([-0.75, 18])  # Adding some margin for better visualization
ax.set_ylim([-0.75, 18])  # Adding some margin for better visualization
x_ticks = np.arange(0, 18, 2.5)
y_ticks = np.arange(0, 18, 2.5)
ax.set_xticks(x_ticks)
ax.set_yticks(y_ticks)
plt.savefig('Scenario 4 plot.pdf')
plt.show()
