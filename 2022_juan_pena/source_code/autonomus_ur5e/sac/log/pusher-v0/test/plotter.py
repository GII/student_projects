from turtle import right
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

log_dir = os.path.dirname(__file__)
print(log_dir)

same_as_training = pd.read_csv(log_dir+"/results_same_training.csv")
robot_random = pd.read_csv(log_dir+"/results_robot_random.csv") 
target_random = pd.read_csv(log_dir+"/results_target_random.csv") 
robot_and_target_random = pd.read_csv(log_dir+"/results_robot_and_target_random.csv") 

data = [same_as_training['reward'].tolist(), robot_random['reward'].tolist(), target_random['reward'].tolist(), robot_and_target_random['reward'].tolist()]
labels=['Igual que en entrenamiento', 'Posición del robot aleatoria',
                         'Posición del objetivo aleatoria','Posicíon del robot y del objetivo aleatoria']
fig, ax = plt.subplots()
ax.set_title('Evaluación experimento 1')
ax.set_ylabel('Recompensa')
ax.set_xticklabels(labels=labels,rotation=20)
ax.boxplot(data,labels=labels)
ax.margins(y=0.25)
fig.subplots_adjust(bottom=0.27,right=0.87,top=0.92)
plt.show()