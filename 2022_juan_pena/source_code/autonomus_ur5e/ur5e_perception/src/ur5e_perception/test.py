import numpy as np
from tensorboard import summary
import tensorflow as tf
import matplotlib.pyplot as plt
import cv2 as cv
import rospkg
import argparse
import os
import pickle
from tensorflow.keras.preprocessing import image_dataset_from_directory
from tensorflow.keras import layers
from tensorflow.keras.datasets import mnist
from tensorflow.keras.models import Model
from keras.preprocessing.image import ImageDataGenerator
import os,shutil

image = cv.imread("/home/juan/tfm_ws/src/autonomus_ur5e/ur5e_perception/data/train/color/image0.png")
width,height,channels =image.shape
resized_length = 256
#crop the image by half
cropped_img = image[0:int(width),0:int(height/2)]
#resize the cropped image
resized_img = cv.resize(cropped_img, (1024,1024), interpolation = cv.INTER_AREA)

cv.imshow('image',resized_img)
cv.waitKey(0)

