from turtle import right
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

log_dir = os.path.dirname(__file__)
print(log_dir)

same_as_training = pd.read_csv(log_dir+"/same_as_training.csv")
object_random = pd.read_csv(log_dir+"/random_object_pose.csv") 
robot_and_objetct_random = pd.read_csv(log_dir+"/random_object_and_random_robot_pose.csv") 

data = [same_as_training['reward'].tolist(), object_random['reward'].tolist(), robot_and_objetct_random['reward'].tolist()]
labels=['Igual que en entrenamiento', 'Posición del objeto aleatoria',
        'Posicíon del robot y del objeto aleatoria']
fig, ax = plt.subplots()
ax.set_title('Evaluación experimento 2')
ax.set_ylabel('Recompensa')
ax.set_xticklabels(labels=labels,rotation=20)
ax.boxplot(data,labels=labels)
ax.margins(y=0.25)
fig.subplots_adjust(bottom=0.27,right=0.87,top=0.92)
plt.show()