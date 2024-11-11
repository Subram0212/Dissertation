import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score

# Replace these values with your confusion matrix data
confusion_matrix = np.array([[69, 1], [0, 30]])

# Define class labels for the confusion matrix
class_labels = ["Class Infeas", "Class Feas"]

# Calculate accuracy, precision, recall, and f1-score
accuracy = (confusion_matrix[0, 0] + confusion_matrix[1, 1]) / (confusion_matrix[0, 0] + confusion_matrix[0, 1] + confusion_matrix[1, 0] + confusion_matrix[1, 1])
precision = confusion_matrix[1, 1] / (confusion_matrix[1, 1] + confusion_matrix[1, 0])
recall = confusion_matrix[1, 1] / (confusion_matrix[1, 1] + confusion_matrix[0, 1])
f1score = (2 * (precision * recall)) / (precision + recall)

# Create a figure and axis
fig, ax = plt.subplots()

# Display the confusion matrix as an image
cax = ax.matshow(confusion_matrix, cmap=plt.cm.Blues)

# Add labels and ticks
plt.title(f'Confusion Matrix\nAccuracy: {accuracy: .2f}, Precision: {precision: .2f}, Recall: {recall: .2f}, F-Score: {f1score: .2f}')
plt.xlabel('True')
plt.ylabel('Predicted')
plt.xticks(range(len(class_labels)), class_labels)
plt.yticks(range(len(class_labels)), class_labels)

# Add text to the matrix cells
for i in range(len(class_labels)):
    for j in range(len(class_labels)):
        plt.text(j, i, str(confusion_matrix[i, j]), va='center', ha='center')

# Add a color bar
plt.colorbar(cax)

# Save the figure as a PNG image
plt.savefig('Confusion matrix and scores/confusion_matrix - GP, scenario 2 and seed 50.png', format='png')

# Display the plot
plt.show()
