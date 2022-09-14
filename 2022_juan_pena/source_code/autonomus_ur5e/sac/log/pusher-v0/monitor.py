from stable_baselines3.common import results_plotter
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3.common.results_plotter import load_results, ts2xy
import pathlib
import os

log_dir = os.path.dirname(__file__)
print(log_dir)
# Helper from the library
#results_plotter.plot_results([log_dir], 100, results_plotter.X_TIMESTEPS, "SAC reacher V0")



def moving_average(values, window):
    """
    Smooth values by doing a moving average
    :param values: (numpy array)
    :param window: (int)
    :return: (numpy array)
    """
    weights = np.repeat(1.0, window) / window
    return np.convolve(values, weights, 'valid')


def plot_results(log_folder, title='Curva de aprendizaje de experimento 1'):
    """
    plot the results

    :param log_folder: (str) the save location of the results to plot
    :param title: (str) the title of the task to plot
    """
    x, y = ts2xy(load_results(log_folder), 'timesteps')
    x2, y2 = ts2xy(load_results("/home/juan/tfm_ws/src/autonomus_ur5e/sac/log/pusher-v1"), 'timesteps')
    y = moving_average(y, window=50)
    y2 = moving_average(y2, window=50)
    # Truncate x
    x = x[len(x) - len(y):]

    fig = plt.figure(title)
    plt.plot(x, y,label="YOLO")
    plt.plot(x,y2,label="Autoencoder")
    plt.xlabel('Numero de instantes de tiempo')
    plt.ylabel('Recompensa')
    plt.title(title)
    plt.legend()
    plt.show()

plot_results(log_dir)