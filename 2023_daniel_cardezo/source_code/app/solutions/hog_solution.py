"""Sistema de reconocimiento de objetos con HOG"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats as stats
from statsmodels.stats.multicomp import pairwise_tukeyhsd, MultiComparison
from sklearn.pipeline import make_pipeline
from sklearn.model_selection import cross_validate
from sklearn.linear_model import LogisticRegression
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import KFold, GridSearchCV
from time import time
import matplotlib.pyplot as plt
from sklearn.pipeline import Pipeline
from skimage.feature import hog

from sklearn.model_selection import train_test_split
from sklearn.model_selection import RandomizedSearchCV
from sklearn.datasets import fetch_lfw_people
from sklearn.metrics import classification_report
from sklearn.metrics import ConfusionMatrixDisplay, classification_report
from sklearn.decomposition import PCA
from sklearn.svm import SVC
from sklearn.utils.fixes import loguniform
import os
import pandas as pd
from openpyxl import Workbook
import csv


def plot_gallery(images, titles, h, w, n_row=3, n_col=4):
    """Helper function to plot a gallery of portraits"""
    plt.figure(figsize=(1.8 * n_col, 2.4 * n_row))
    plt.subplots_adjust(bottom=0, left=0.01, right=0.99, top=0.90, hspace=0.35)
    for i in range(n_row * n_col):
        plt.subplot(n_row, n_col, i + 1)
        plt.imshow(images[i].reshape((h, w)), cmap=plt.cm.gray)
        plt.title(titles[i], size=12)
        plt.xticks(())
        plt.yticks(())


def title(y_pred, y_test, target_names, i):
    pred_name = target_names[y_pred[i]].rsplit(" ", 1)[-1]
    true_name = target_names[y_test[i]].rsplit(" ", 1)[-1]
    return "predicted: %s\ntrue:      %s" % (pred_name, true_name)

#Obtencion del dataset y de los paths 
DATA_DIR = "C:/data_dir"
RES_DIR = "C:/datasets"
CLASSIFIER = "SVC"
TEST_SIZE = 0.25


# lectura de los datos Labeled Faces in the Wild (LFW) (maybe i need to select the folder i desire instead of reading this)

lfw_people = fetch_lfw_people(min_faces_per_person=50, resize=1)
images = lfw_people.images
n_samples = images.shape[0]
n_features = lfw_people.data.shape[1]
t = lfw_people.target
target_names = lfw_people.target_names
# n_classes = lfw_people.target_names.shape[0]
# x_train, x_val, t_train, t_val = train_test_split(
#     x, t, test_size=0.25, random_state=42
# )

#Entrenamiento 

# Here we made the staction of characteristics using HOG

hog_images = []
x = []
for image in images:
    fd, hog_image = hog(
        image, orientations=8, pixels_per_cell=(16, 16), cells_per_block=(1, 1), visualize=True, feature_vector=True
    )
    x.append(fd)
    hog_images.append(hog_image)
x = np.array(x)
x = StandardScaler().fit_transform(x)
hog_images = np.array(hog_images)
n_samples, h, w = hog_images.shape


# here we separate the dat in test train and valid (so it means that the data folder can be desorganized?)

x_train, x_val, t_train, t_val = train_test_split(x, t, test_size=TEST_SIZE, random_state=42)


# now comes theh selection and training of the SVM, we implemented two to select, SVN and ANN

if(CLASSIFIER == "SVC"):

    # SVC
    svc_model = SVC()
    svc_model.fit(x_train, t_train)

    # after the training we should select the hyperparameters for each model

    # hyperparameter selection for SVC

    param_grid = {"C": [1, 10, 100, 1000], "gamma": [1, 0.1, 0.001, 0.0001], "kernel": ["linear", "rbf"]}
    grid = GridSearchCV(SVC(), param_grid, refit=True, scoring="accuracy")
    svc_results = grid.fit(x_train, t_train)

# ANN

if(CLASSIFIER == "ANN"):

    ANN_model = MLPClassifier(
        hidden_layer_sizes=(40,),
        max_iter=8,
        alpha=1e-4,
        solver="sgd",
        verbose=10,
        random_state=1,
        learning_rate_init=0.2,
    )
    ANN_model.fit(x_train, t_train)

    # after the training we should select the hyperparameters for each model

    # hyperparameter selection for ANN

    param_grid = {
        "hidden_layer_sizes": [(50, 50, 50), (50, 100, 50), (100,)],
        "activation": ["tanh", "relu"],
        "solver": ["sgd", "adam"],
        "alpha": [0.0001, 0.05],
        "learning_rate": ["constant", "adaptive"],
    }
    grid = GridSearchCV(MLPClassifier(), param_grid, refit=True, scoring="accuracy")
    ann_results = grid.fit(x_train, t_train)



#Result visualization 
# finally, the results with the best model are `rovided`

if(CLASSIFIER == "ANN"):
    print("Predicting people's names on the test set")
    y_pred = ann_results.best_estimator_.predict(x_val)
    report = (classification_report(t_val, y_pred, target_names=target_names))
    ConfusionMatrixDisplay.from_estimator(
        ann_results.best_estimator_, x_val, t_val, display_labels=target_names, xticks_rotation="vertical"
    )

    os.chdir(RES_DIR)
    in_lines = report.splitlines()
    #  out_lines = len(in_lines)
    #  loses = int(out_lines)
    #  i = 0
    #  while i < loses:
    #      in_lines[i] = in_lines[i].split()
    #      print(in_lines)
    #      i = i + 1

    df = pd.DataFrame(in_lines)
    data = df.to_csv("Face_recognition_classification_report.csv", index=True)

    wb = Workbook()
    ws = wb.active
    with open("Face_recognition_classification_report.csv", "r") as f:
        for row in csv.reader(f):
            ws.append(row)
    wb.save("Face_recognition_classification_report.xlsx")


    plt.tight_layout()
    plt.show()


if(CLASSIFIER == "SVC"):
    print("Predicting people's names on the test set")
    y_pred = svc_results.best_estimator_.predict(x_val)
    report = (classification_report(t_val, y_pred, target_names=target_names))
    ConfusionMatrixDisplay.from_estimator(
        svc_results.best_estimator_, x_val, t_val, display_labels=target_names, xticks_rotation="vertical"
    )

    os.chdir(RES_DIR)
    in_lines = report.splitlines()
    #  out_lines = len(in_lines)
    #  loses = int(out_lines)
    #  i = 0
    #  while i < loses:
    #      in_lines[i] = in_lines[i].split()
    #      print(in_lines)
    #      i = i + 1

    df = pd.DataFrame(in_lines)
    data = df.to_csv("Face_recognition_classification_report.csv", index=True)

    wb = Workbook()
    ws = wb.active
    with open("Face_recognition_classification_report.csv", "r") as f:
        for row in csv.reader(f):
            ws.append(row)
    wb.save("Face_recognition_classification_report.xlsx")


    plt.tight_layout()
    plt.show()
