# Prueba sensor HC-SR501

from gpiozero import MotionSensor
from time import sleep

pir = MotionSensor(4)

while True:
    if pir.motion_detected:
        print("Alerta intruso!!")
        sleep(0.5)
