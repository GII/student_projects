from turtle import right
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

log_dir = os.path.dirname(__file__)
print(log_dir)


robot_and_objetct_random = pd.read_csv(log_dir+"/random_object_and_random_robot_pose.csv") 

data = robot_and_objetct_random['reward'].tolist()
labels=['Posicíon del robot y del objeto aleatoria']
fig, ax = plt.subplots()
ax.set_title('Evaluación experimento 3')
ax.set_ylabel('Recompensa')
ax.boxplot(data,labels=labels)
ax.margins(y=1)
fig.subplots_adjust(bottom=0.27,right=0.87,top=0.92)
plt.show()