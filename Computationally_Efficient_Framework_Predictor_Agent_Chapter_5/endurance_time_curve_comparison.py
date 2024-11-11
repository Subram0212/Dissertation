import matplotlib.pyplot as plt
import numpy

x1 = [0, 45, 60, 101, 116, 175, 214, 229, 261, 276, 300, 315, 404, 437, 452, 487, 502, 544, 560]
y1 = [0, 45, 0, 41, 0, 0, 39, 0, 32, 0, 34, 0, 0, 33, 0, 35, 0, 42, 0]

x2 = [0, 50, 65, 102, 117, 180, 217, 232, 260, 275, 319, 334, 405, 455, 470, 505, 520, 562, 577]
y2 = [0, 50, 0, 37, 0, 0, 37, 0, 28, 0, 44, 0, 0, 50, 0, 35, 0, 42, 0]

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(x1, y1, color='r', label='Simulation', linestyle='--')
ax.plot(x2, y2, label='Experiment')

plt.xlabel('Experiment progress (s)')
plt.ylabel('UAV Endurance (s)')
plt.ylim([0, 70])
plt.legend(loc='upper right')
ax.plot([0, 600], [50, 50], 'k-')
plt.savefig('Endurance plot re-planning with dynamic changes 3_18.pdf')
plt.show()
