# Prueba sensor HC-SR04.

from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(6, 5)

while True:
    print("La distancia al objeto más cercano es", sensor.distance, "m")
    sleep(1)

    if sensor.distance <= 0.2:
        print("Intruso")
