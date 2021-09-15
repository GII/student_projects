# Prueba camara

from picamera import PiCamera
from time import sleep
from datetime import datetime

camera = PiCamera()
camera.capture("/home/pi/image" + str(datetime.now()) + ".jpg")
