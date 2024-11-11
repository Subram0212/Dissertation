import numpy as np
import pandas as pd
import overall_PS_multiUGVstops_UGVUAVmoving as PS
import minimum_set_cover_UGV_hyperparam_refined
import ugv_distance_calc_auto
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score, confusion_matrix, roc_curve, auc
import matplotlib.pyplot as plt

# Initialize lists to store the data
X = []
y = []

# Open the file for reading
with open('output_111.txt', 'r') as file:
    for line in file:
        values = line.split()
        if len(values) >= 5:
            X.append(list(map(int, values[:4])))
            y.append(float(values[4]))

y_actual = []

population_dict = {}
loc_population_dict = {}
glob_population_dict = {}
overall_population_dict = []
overall_conv_crita_list = []
total_global_opt_fn_eval = 0
total_local_opt_fn_eval = 0
n_samples = 40
num_uavs = 1
"""Ensure that the changes that you make here is properly reflected in the 'genetic_algorithm' module"""
# Consolidating possible UGV stop location pairs into a hash table (dictionary)
ordered_pts_from_depot = pd.read_excel('ARL corridor points ordered DepotB.xlsx', engine='openpyxl')
ordered_pts_rest_df = pd.read_excel('ARL corridor points ordered.xlsx', engine='openpyxl')
ordered_pts_from_depot = ordered_pts_from_depot.values.tolist()
ordered_pts_rest = ordered_pts_rest_df.values.tolist()
ordered_pts_from_depot = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_from_depot]
ordered_pts_rest = [(round(m[0], 2), round(m[1], 2)) for m in ordered_pts_rest]

reversed_ordered_pts_rest = list(reversed(ordered_pts_rest))
ugv_stops_dict = {}
ugv_Stops = minimum_set_cover_UGV_hyperparam_refined.main(ordered_pts_rest_df, 'Case_Study_scenario.csv')

for i in range(len(ugv_Stops)):
    ugv_stops_dict[i+2] = ugv_Stops[i]

for n in range(len(X)):
    param_list = X.pop(0)
    fitness_val, max_time, depotb_vel, ugv_vel, track_node_age, route_dict, route_dict2, route_dict3, sp_feas_list, ugv_list, total_cost = PS.persistent_surveillance(param_list, num_uavs, ugv_stops_dict)
    if total_cost > 1_000_000:
        y_actual.append(0)
    else:
        y_actual.append(1)

y_pred_arr = np.array(y)
y_actual_arr = np.array(y_actual)
print(f"Predicted infeasibility list: {y}")
print(f"Actual infeasibile output list: {y_actual}")
print(f"The accuracy score for predicted vs actual responses for infeasible predicted output is: {accuracy_score(y_actual_arr, y_pred_arr)*100}%")
actual_feasibility_list = [i for i in y_actual if i == 1]
predicted_feasibility_list = [i for i in y if i == 1]
print(f"Predicted feasible output count is: {len(predicted_feasibility_list)} and total output length is: {len(y)}")
print(f"Actual feasible output count is: {len(actual_feasibility_list)} and total output length is: {len(y_actual)}")

# Calculate the confusion matrix
conf_matrix = confusion_matrix(y_actual, y)

# Calculate precision, recall, f-score, and accuracy
precision = precision_score(y_actual, y)
recall = recall_score(y_actual, y)
f1 = f1_score(y_actual, y)
accuracy = accuracy_score(y_actual, y)
fpr, tpr, thresholds = roc_curve(y_actual, y)

roc_auc = auc(fpr, tpr)

# Create a figure and axis
fig, ax = plt.subplots()

# Display the confusion matrix as an image
cax = ax.matshow(conf_matrix, cmap=plt.cm.Oranges, vmin=0, vmax=100)

# # Loop over the data and create a text annotation for each square
# for i in range(conf_matrix.shape[0]):
#     for j in range(conf_matrix.shape[1]):
#         # Determine text color
#         color = "white" if cax.norm(conf_matrix[i, j]) > threshold else "black"
#         # Apply bold font weight for better visibility
#         ax.text(j, i, str(conf_matrix[i, j]), va='center', ha='center', color=color, fontweight='bold')

# Add labels and ticks
plt.title(f'Confusion Matrix\nAccuracy: {accuracy:.2f}, Precision: {precision:.2f}, Recall: {recall:.2f}, F-Score: {f1:.2f}')
plt.xlabel('Predicted UGV-UAV solution feasibility')
plt.ylabel('Actual UGV-UAV solution feasibility')
plt.xticks(range(2))
plt.yticks(range(2))

# Add text to the matrix cells
for i in range(2):
    for j in range(2):
        plt.text(j, i, str(conf_matrix[i, j]), va='center', ha='center')

plt.savefig('Confusion matrix and scores/confusion_matrix - kNN, scenario 1 and seed 111.pdf', format='pdf')

# Add a color bar
plt.colorbar(cax)
# plt.savefig('Confusion matrix and scores/confusion_matrix - kNN, scenario 1 and seed 111.png', format='png')
plt.figure()
plt.plot(fpr, tpr, color='darkorange', lw=2, label='ROC curve (area = %0.2f)' % roc_auc)
plt.plot([0, 1], [0, 1], color='navy', lw=2, linestyle='--')
plt.xlim([-0.05, 1.0])
plt.ylim([-0.05, 1.05])
plt.xlabel('False Positive Rate')
plt.ylabel('True Positive Rate')
plt.title('Receiver Operating Characteristic')
plt.legend(loc="lower right")
# Save the figure as a PDF image
plt.savefig('Confusion matrix and scores/ROC curve - kNN, scneario 1 and seed 111.pdf', format='pdf')
# Display the plot
plt.show()
