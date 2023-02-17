"""Sistema de reconocimiento de objetos con Google Vision API"""

# Import libraries
from cProfile import label
import os

import cv2
import time
import sys
from skimage.io import imsave
from google.cloud import vision
import datetime
from email.mime import image
import io

""" Hyperparameters and Constants

Here, we define wether to train the model or not and for how many epochs to train for.

"""

# nothing to do here, parameters defined locally

#Datos usuario
""" 
    Here, in this project, the images used are taken directly from the laptop camera,
    so we do not need to select a folder to get the images from.

    nevertheless, we need to open the laptop camera and take frames from it. 
"""

OBJECT_1 = ""
OBJECT_2 = ""
RES_DIR = ""
DATA_DIR = ""

# function in order to capture webcam images to lately be processed
def webcam_capture_test(camera_device):

    os.chdir(RES_DIR)
    for frame in os.listdir(DATA_DIR):
        local = DATA_DIR + "\\" + frame
        with io.open(local, "rb") as image_file:
            content = image_file.read()
        frame_processing(content)

        # 2000 ms delay --> wait for 2 seconds
        key = cv2.waitKey(2000) & 0xFF
        # If the key "q" is pressed, the loop
        # stops before the defined diration time
        if key > 0 and chr(key) == "q":
            break

#Entrenamiento
"""
    The frames obtained from the camera, needs to be processed.
    In the function below, also the targets to detect in the process are defined,
    It is important to mention that the OpenCV images are in BGR format, so in this
    function it is important to convert the images to RGB format.
"""


def frame_processing(content):

    targets = [OBJECT_1, OBJECT_2, "Vehicle"]

    labels = detect_labels(content)
    process_labels(labels, frame=content, targets=targets)

""" 
    The next step (subfunction in the frame treatment above) is to detect the labels 
    in the treated frames. For this step, we call the Google Cloud Vision API.
"""


def detect_labels(content):
    """Set google credentials"""
    credential = r"C:\prueba_python\buoyant-road-375718-d3c4bf268d24.json"

    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = credential
    """Detects labels in the file."""
    from google.cloud import vision
    import io

    client = vision.ImageAnnotatorClient()

    # with io.open(path, 'rb') as image_file:
    #     content = image_file.read()

    image = vision.Image(content=content)

    response = client.object_localization(image=image)
    labels = response.localized_object_annotations

    print("Labels:")
    # loop in order to print all the labels detected in the terminal
    for label in labels:
        print(label)
    # if there´s any problem to be reported
    if response.error.message:
        raise Exception(
            "{}\nFor more info on error messages, check: "
            "https://cloud.google.com/apis/design/errors".format(response.error.message)
        )

    return labels

#Visualización de resultados
""" 
    Here the program analyzes the objects detected in the image and if a person or a vehicle are detected,
    it saves the result in a .txt file
    NOTE: In the function below, the labels detected before are processed.
    If the label fits on the target, and the score is higher than 0.5, 
    the label will be saved.
"""


def process_labels(labels, frame, score=0.5, targets=[]):
    object_2_count = 0
    object_1_count = 0
    results = dict()
    results["object_2_count"] = object_2_count
    results["object_1_count"] = object_1_count
    now = datetime.datetime.now()
    # print(labels.name)
    for label in labels:

        for target in targets:
            # print(target.score)
            if label.name == target and label.score >= score:

#                imsave(
#                    str(now.year)
#                    + "_"
#                    + str(now.month)
#                    + "_"
#                    + str(now.day)
#                    + "_"
#                    + str(now.hour)
#                    + "_"
#                    + str(now.minute)
#                    + "_"
#                    + str(now.second)
#                    + ".jpg",
#                    frame,
#                )
                if label.name == OBJECT_1:
                    object_1_count = object_1_count + 1
                else:
                    object_2_count = object_2_count + 1

                results["object_2_count"] = object_2_count
                results["object_1_count"] = object_1_count

    log_info(results)

    return results


""" 
    This function saves in a .txt file the results provided by the function above.
    NOTE : It is programmed in order to save only persons and vehicles.
"""


def log_info(results):

    os.chdir(RES_DIR)
    with open("log.txt", "a") as f:
        now = datetime.datetime.now()
        str_to_write = (
            "date: "
            + str(now.day)
            + "/"
            + str(now.month)
            + "/"
            + str(now.year)
            + " "
            + str(OBJECT_1)
            + ": "
            + str(results["object_1_count"])
            + " "
            + str(OBJECT_2)
            + ": "
            + str(results["object_2_count"])
        )
        f.write(str_to_write)
        f.write("\n")


#Bucle principal de la aplicación
""" 
    Main function, selects the peripheral to use for image capture.
    Calls the capture function and then the processing of the captured images starts.
    
"""


if __name__ == "__main__":
    camera_device = 0
    if len(sys.argv) > 1:
        camera_device = int(sys.argv[1])

    webcam_capture_test(camera_device)
