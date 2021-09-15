# Prueba sensor IMU

from sense_hat import SenseHat
from time import sleep

sense = SenseHat()

sense.set_imu_config(True, True, True)

old_orientation = sense.get_orientation_degrees()
old_roll = old_orientation["roll"]
old_pitch = old_orientation["pitch"]
old_yaw = old_orientation["yaw"]

while True:
    orientation = sense.get_orientation_degrees()
    roll = orientation["roll"]
    pitch = orientation["pitch"]
    yaw = orientation["yaw"]

    if (abs(old_roll - roll) > 3) or (abs(old_pitch - pitch) > 3) or (abs(old_yaw - yaw) > 3):

        print("¡¡¡ALERTA MOVIMIENTO!!!")
        print(f"roll vale {roll}, pitch vale {pitch} y yaw vale {yaw}")

        sleep(0.5)

    old_roll = roll
    old_pitch = pitch
    old_yaw = yaw
