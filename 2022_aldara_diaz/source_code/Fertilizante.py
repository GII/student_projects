# CÓDIGO PARA PUBLICAR MENSAJES EN UN TOPIC DE AWS MEDIANTE RASPBERRY PI

from random import randrange
from pyowm import OWM
from awscrt import io, mqtt
from awsiot import mqtt_connection_builder
import socket
import json


# Conexión MQTT
def setup_mqtt_connection():
    endpoint = "xxxxxxxxxxxxx-xxx.iot.eu-west-1.amazonaws.com"
    client_id = socket.gethostname()
    # An event-loop is a thread for doing async work, such as I/O
    event_loop_group = io.EventLoopGroup(num_threads=1)
    # DNS host resolver to use
    host_resolver = io.DefaultHostResolver(event_loop_group)
    # Handles creation and setup of client socket connections
    client_bootstrap = io.ClientBootstrap(event_loop_group, host_resolver)
    mqtt_connection = mqtt_connection_builder.mtls_from_path(
        endpoint=endpoint,
        client_id=client_id,
        cert_filepath="/home/pi/TFG/certificate.pem.crt",
        pri_key_filepath="/home/pi/TFG/private.pem.key",
        ca_filepath="/home/pi/TFG/AmazonRootCA1.pem",
        client_bootstrap=client_bootstrap,
    )
    print(f"Conectando a {endpoint} con id {client_id}...")
    mqtt_connection.connect().result()
    print("Conectado")
    return mqtt_connection

# Conseguir información del nivel de fertilizante disponible
def get_info():
    # Se adquiere un valor aleatorio entre 0 y 25000 L
    nivel = str(randrange(0, 25000))
    return nivel

# Publicar la información en un topic de AWS
def publish(mqtt_connection):
    nivel = get_info()
    # Transformación del mensaje a JSON para poder enviar al topic correspondiente
    mensaje = json.dumps(
        {
            "Nivel_fertilizante": nivel,
        }
    )
    # Publicación en el topic "Datos_Raspberry"
    mqtt_connection.publish(topic="Datos_Raspberry",
                            payload=mensaje, qos=mqtt.QoS.AT_LEAST_ONCE)
    print(f"El nivel de fertilizante es {nivel} L")

# Desconexión MQTT
def disconnect_mqtt_broker(mqtt_connection):
    print("Desconectando...")
    mqtt_connection.disconnect().result()
    print("Desconectado")


def main():
    mqtt_connection = setup_mqtt_connection()
    publish(mqtt_connection)
    disconnect_mqtt_broker(mqtt_connection)


if __name__ == "__main__":
    main()