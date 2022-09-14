from enum import auto
import numpy as np
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


class Autoecoder():

    def __init__(self):      

        self.package_path = "/home/juan/tfm_ws/src/autonomus_ur5e/ur5e_perception"
        

    def _load_data(self,type=""):

        if type=="color":
            train_list = os.listdir(self.package_path+"/data/train/color/")
            test_list = os.listdir(self.package_path+"/data/test/color/")
            train_data = []
            test_data = []
            for image in train_list:
                train_data.append(cv.imread(self.package_path+"/data/train/color/"+image))
            for image in test_list:
                test_data.append(cv.imread(self.package_path+"/data/test/color/"+image))
            
        elif type =="depth":
            train_list = os.listdir(self.package_path+"/data/train/depth/")
            test_list = os.listdir(self.package_path+"/data/test/depth/")
            train_data = []
            test_data = []
            for image in train_list:
                train_data.append(cv.imread(self.package_path+"/data/train/depth/"+image))
            for image in test_list:
                test_data.append(cv.imread(self.package_path+"/data/test/depth/"+image))

        train_data = np.array(train_data)
        test_data = np.array(test_data)

        return train_data,test_data


    def _preprocess(self,array):
        """
        Normalizes the supplied dataset and reshapes it into the appropriate format.
        """
        list=[]
        for image in array:

            width,height,channels =image.shape
            resized_length = 1024    #FROM 256 TO 1024
            #crop the image by half
            cropped_img = image[0:int(width),0:int(height/2)]
            #resize the cropped image
            resized_img = cv.resize(cropped_img, (resized_length,resized_length), interpolation = cv.INTER_AREA)
            # Normalizing the images to [0,1]
            input_image = (resized_img /255.0) 

            list.append(resized_img)

        list = np.array(list)

        return list
    def display(self,array1, array2,save=False,rescale=False,name="comparison"):
        """
        Displays ten random images from each one of the supplied arrays.
        """
        

        n = 10
        if array1.size >= array2.size:
            indices = np.random.randint(len(array2), size=n)
        else:
            indices = np.random.randint(len(array1), size=n)

        images1 = array1[indices, :]
        images2 = array2[indices, :]

        plt.figure(figsize=(20, 4))
        for i, (image1, image2) in enumerate(zip(images1, images2)):
            if rescale:
                image1 = image1*255.0
                image2 = image2*255.0
            image1 = cv.resize(image1,(250,250),interpolation = cv.INTER_AREA)
            image2 = cv.resize(image2,(250,250),interpolation = cv.INTER_AREA)
            # image2 = np.rint(image2)
            # print(image1.shape)
            # print(image1)
            ax = plt.subplot(2, n, i + 1)
            plt.imshow(image1)
            plt.gray()
            ax.get_xaxis().set_visible(False)
            ax.get_yaxis().set_visible(False)

            ax = plt.subplot(2, n, i + 1 + n)
            plt.imshow(image2.astype('uint8'))
            plt.gray()
            ax.get_xaxis().set_visible(False)
            ax.get_yaxis().set_visible(False)

        if save :
            plt.savefig(self.package_path+"/results/figures/"+name+".png")
        plt.show()
        

    def _autoencoder(self):
        encoding_dim = 256
 

        inputs = layers.Input(shape=(1024, 1024, 3))

        input = layers.Input(shape=(1024, 1024, 3))
        x = layers.Conv2D(32, (7, 7), activation="relu", padding="same")(input)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) #512
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(32, (7, 7), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) #256
        x = layers.LeakyReLU()(x)
        #/-------------------- BELOW IS THE ORIGINAL CODE -------------------------------///
        # input = layers.Input(shape=(256, 256, 3))
        x = layers.Conv2D(32, (7, 7), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) #128
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(64, (5, 5), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) #64
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(128, (3, 3), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) #32
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(256, (3, 3), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) #16
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(512, (3, 3), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) # 8
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(512, (3, 3), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) # 4
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(512, (3, 3), activation="relu", padding="same")(x)
        x = layers.MaxPooling2D((2, 2), padding="same")(x) # 2
        x = layers.LeakyReLU()(x)
        x = layers.Conv2D(1024, (3, 3), activation="relu", padding="same")(x) 
        x = layers.MaxPooling2D((2, 2), padding="same")(x) # 1 cambie a 1024
        x = layers.LeakyReLU()(x)
        shape = x.shape[1:]

        #//--------latent space----------//
        x = layers.Flatten()(x)
        x = layers.Dense(encoding_dim)(x)
        z = layers.LeakyReLU()(x)
        self._encoder = Model(input,z,name="encoder")
        self._encoder.make_predict_function()
        #//--------latent space----------//

        # Decoder

        #decoder cambiamos de leaky relu a relu 
        latent_inputs = layers.Input(shape=(encoding_dim,))
        x = layers.Dense(np.prod(shape))(latent_inputs)
        x = layers.LeakyReLU()(x)
        x = layers.Reshape(shape)(x)                                                #1
        x = layers.UpSampling2D((2,2))(x)                                           #2        
        x = layers.Conv2DTranspose(512,(3,3),activation="relu",padding="same")(x)
        x = layers.LeakyReLU()(x)
        x = layers.UpSampling2D((2,2))(x) 
        x = layers.Conv2DTranspose(512,(3,3),activation="relu",padding="same")(x)          
        x = layers.UpSampling2D((2,2))(x) 
        x = layers.Conv2DTranspose(512,(3,3),activation="relu",padding="same")(x)
        x = layers.LeakyReLU()(x)           
        x = layers.UpSampling2D((2,2))(x)                                           #16 
        x = layers.Conv2DTranspose(256,(3,3),activation="relu",padding="same")(x)  
        x = layers.LeakyReLU()(x)         
        x = layers.UpSampling2D((2,2))(x)                                           #32
        x = layers.Conv2DTranspose(128,(3,3),activation="relu",padding="same")(x) 
        x = layers.LeakyReLU()(x)          
        x = layers.UpSampling2D((2,2))(x)                                           #64
        x = layers.Conv2DTranspose(64,(3,3),activation="relu",padding="same")(x)
        x = layers.LeakyReLU()(x)           
        x = layers.UpSampling2D((2,2))(x)                                           #128
        x = layers.Conv2DTranspose(32,(5,5),activation="relu",padding="same")(x) 
        x = layers.LeakyReLU()(x)          
        x = layers.UpSampling2D((2,2))(x)                                           #256
        x = layers.Conv2DTranspose(32,(7,7),activation="relu",padding="same")(x)     #CHANGED 3 TO 32
       
    #/-------------------- ABOVE IS THE ORIGINAL CODE -------------------------------///
        x = layers.LeakyReLU()(x)          
        x = layers.UpSampling2D((2,2))(x)                                           #512
        x = layers.Conv2DTranspose(32,(7,7),activation="relu",padding="same")(x) 
        x = layers.LeakyReLU()(x)          
        x = layers.UpSampling2D((2,2))(x)                                           #1024
        x = layers.Conv2DTranspose(3,(7,7),activation="relu",padding="same")(x) 

        self._decoder=Model(latent_inputs,x,name="decoder")  
        loss = 'mean_squared_error' 


        # Autoencoder
        _model = Model(inputs, self._decoder(self._encoder(inputs)))
        _model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=2e-5),
                         loss=loss)
        return _model
   

    def train(self,type=""):
        batch_size = 20

        train,test = self._load_data(type=type)

        train = self._preprocess(train)
        test = self._preprocess(test)


        
        self.autoencoder = self._autoencoder()

        self.history = self.autoencoder.fit(
            x=train,
            y=train,
            epochs=200,
            batch_size=batch_size,
            shuffle=True,
            validation_data=(test, test))

    def plot_history(self,save=False,name="loss"):

        loss = self.history.history['loss']
        val_loss = self.history.history['val_loss']
        epochs = range(len(loss))
        plt.plot(epochs, loss, 'b', label='Error en datos de entranmiento')
        plt.plot(epochs, val_loss, 'r', label='Error en datos de validación')
        plt.title('Error en datos de entrenamiento y validación')
        plt.legend()
        
        if save :
            plt.savefig(self.package_path+"/results/figures/"+name+".png")
        else:
            plt.show()

    def save_model(self,type,name="model"):

        if type=="color":
            self._encoder.save(self.package_path+"/results/model/encoder/"+name+"color.h5")
            self.autoencoder.save(self.package_path+"/results/model/autoencoder/"+name+"color.h5")

        elif type =="depth":
            self._encoder.save(self.package_path+"/results/model/encoder/"+name+"depth.h5")
            self.autoencoder.save(self.package_path+"/results/model/autoencoder/"+name+"depth.h5")




if __name__ == '__main__':
    autoencoder = Autoecoder()

    train,test=autoencoder._load_data("depth")
    test=autoencoder._preprocess(test)
    model = tf.keras.models.load_model('/home/juan/tfm_ws/src/autonomus_ur5e/ur5e_perception/results/model/autoencoder/depth.h5')
    

    predictions = model.predict(test)   

    autoencoder.display(test,predictions,save=True,name="depth_comparison")


    




