# Prueba sistema global

from sense_hat import SenseHat
from picamera import PiCamera
from time import sleep
from datetime import datetime
import boto3
import json
from gpiozero import MotionSensor
from gpiozero import DistanceSensor
from awscrt import io, mqtt
from awsiot import mqtt_connection_builder
import socket


def init_sensehat():
    """Función para inicializar la sense"""
    sense = SenseHat()
    sense.set_imu_config(True, True, True)
    return sense


def get_orientation(sense):
    """Función para obtener los valores de la orientación"""
    orientation = sense.get_orientation_degrees()
    roll = orientation["roll"]
    pitch = orientation["pitch"]
    yaw = orientation["yaw"]
    return roll, pitch, yaw


def screen_alarm():
    """Función para escribir por pantalla el mensaje de alerta"""
    print("¡ALERTA MOVIMIENTO!")
    sleep(0.5)


def show_leds(sense):
    """Función para mostrar en la pantalla LED una señal de alarma"""
    red = (255, 0, 0)
    white = (255, 255, 255)

    sense.clear(red)
    sleep(0.5)
    sense.clear(white)
    sleep(0.5)
    sense.clear()


def detect_faces(photo, ultima_imagen):
    """Función para detectar y analizar rostros"""
    client = boto3.client("rekognition", aws_access_key_id="XXX", aws_secret_access_key="XXX", region_name="eu-west-1")

    with open("/home/pi/tfg/" + str(ultima_imagen) + ".jpg", "rb") as image:
        response = client.detect_faces(Image={"Bytes": image.read()}, Attributes=["ALL"])

    print("Detected faces for " + photo)
    for faceDetail in response["FaceDetails"]:
        print(
            "The detected face is between "
            + str(faceDetail["AgeRange"]["Low"])
            + " and "
            + str(faceDetail["AgeRange"]["High"])
            + " years old"
        )
        print("Here are the other attributes:")
        print(json.dumps(faceDetail, indent=4, sort_keys=True))

    return len(response["FaceDetails"])


def capture_frame(camera, ultima_imagen):
    """Función para capturar fotos"""
    ultima_imagen = ultima_imagen + 1
    camera.capture("/home/pi/tfg/" + str(ultima_imagen) + ".jpg")
    return ultima_imagen


def detection(camera, ultima_imagen, sense):
    """Función para ejecutar acciones cuando hay una deteccion"""
    screen_alarm()

    for _ in range(3):
        show_leds(sense)

    ultima_imagen = capture_frame(camera, ultima_imagen)

    photo = "photo"
    face_count = detect_faces(photo, ultima_imagen)
    print("Faces detected: " + str(face_count))

    mqtt_connection = setup_mqtt_connection()
    publish_raspberry_data(mqtt_connection)
    sleep(5)
    disconnect_mqtt_broker(mqtt_connection)


def setup_mqtt_connection():
    """Función para establecer conexión MQTT"""
    endpoint = "a2kxagdhlagjlt-ats.iot.eu-west-1.amazonaws.com"
    client_id = socket.gethostname()
    event_loop_group = io.EventLoopGroup(num_threads=1)
    host_resolver = io.DefaultHostResolver(event_loop_group)
    client_bootstrap = io.ClientBootstrap(event_loop_group, host_resolver)
    mqtt_connection = mqtt_connection_builder.mtls_from_path(
        endpoint=endpoint,
        client_id=client_id,
        cert_filepath="/home/pi/certs/XXX-certificate.pem.crt",
        pri_key_filepath="/home/pi/certs/XXX-private.pem.key",
        ca_filepath="/home/pi/certs/AmazonRootCA1 (1).pem",
        client_bootstrap=client_bootstrap,
    )
    print(f"Conectando a {endpoint} con id {client_id}...")
    mqtt_connection.connect().result()
    print("Conectado")
    return mqtt_connection


def publish_raspberry_data(mqtt_connection):
    """Función para publicar datos en topic"""
    ahora = datetime.now()
    fecha = ahora.strftime("%d/%m/%Y")
    hora = ahora.strftime("%H:%M:%S")
    raspberry_n = "Raspberry_1"
    zona_n = "zona_1"

    mensaje = json.dumps({"Fecha": fecha, "Hora": hora, "Raspberry_n": raspberry_n, "Zona_n": zona_n})
    mqtt_connection.publish(topic="raspberry_data", payload=mensaje, qos=mqtt.QoS.AT_LEAST_ONCE)
    print("Se ha detectado un posible intruso")


def disconnect_mqtt_broker(mqtt_connection):
    """Función para cerrar conexión MQTT"""
    print("Desconectando...")
    mqtt_connection.disconnect().result()
    print("Desconectado")


def main():
    """Programa principal"""

    camera = PiCamera()

    pir = MotionSensor(4)

    sensor = DistanceSensor(6, 5)

    sense = init_sensehat()

    old_roll, old_pitch, old_yaw = get_orientation(sense)

    a = 5
    b = 345
    ultima_imagen = 0

    while True:

        roll, pitch, yaw = get_orientation(sense)

        if (
            (
                (abs(old_roll - roll) > a and abs(old_roll - roll) < b)
                or (abs(old_pitch - pitch) > a and abs(old_pitch - pitch) < b)
                or (abs(old_yaw - yaw) > a and abs(old_yaw - yaw) < b)
            )
            or pir.motion_detected
            or sensor.distance <= 0.2
        ):

            detection(camera, ultima_imagen, sense)

        old_roll, old_pitch, old_yaw = roll, pitch, yaw


if __name__ == "__main__":
    main()
