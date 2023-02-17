"""Sistema de reconocimiento de objetos con YOLOv5"""

# importation of the needed libraries

import os
import glob as glob
import matplotlib.pyplot as plt
import cv2
import requests
import random
import numpy as np
import subprocess

# Information from the user

SEED = 
EPOCHS = 
RES_DIR = ""
DATA_DIR = ""
TRAIN_DATA = DATA_DIR + "\\train"
TEST_DATA = DATA_DIR + "\\test"
VAL = DATA_DIR + "\\valid"
INFERENCE = ""
TRAIN = True

np.random.seed(SEED)


#Entrenamiento
glob.glob("train/images/*")

glob.glob("train/labels/*")

process = subprocess.call(
    [
        "python yolov5\\train.py",
        f"--data {TRAIN_DATA}",
        "--weights yolov5s.pt",
        "--img 640",
        "--epochs 25",
        "--batch-size 16",
    ],
    shell=True,
)

#Inference
"""### Inference
In this section, we will carry out inference on unseen images from the inference path. 

"""### Inference on Images

# Helper function for inference on images.
def inference(INFERENCE):
    # Inference on images.
    process_infer = subprocess.call(
    [
        "python yolov5\\detect.py",
        f"--data {INFERENCE}",
        "--weights yolov5s.pt",
        "--img 640",
        "--epochs EPOCHS",
        "--batch-size 16",
    ],
    shell=True,
)
    return INFERENCE

# Inference on images.
IMAGE_INFER_DIR = inference(INFERENCE)


	
