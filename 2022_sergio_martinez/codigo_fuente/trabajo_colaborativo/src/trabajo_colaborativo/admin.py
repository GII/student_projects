from trabajo_colaborativo.msg import Espera
from trabajo_colaborativo.msg import Cilindro
from trabajo_colaborativo.msg import Aparcado
import rospy

class Administrador:
    def __init__(self):
        self.robobo1 = "/robot/rob1"
        self.robobo2 = "/robot/rob2"
        self.brazo1 = "brazo1"
        self.brazo2 = "brazo2"
        self.cinta = "cinta"
        self.cilindro_sub = rospy.Subscriber("/cilindro", Cilindro, self.callback_cilindro)
        self.aparcado_sub = rospy.Subscriber("/aparcado", Aparcado, self.callback_aparcado)
        self.cilindro_pub = rospy.Publisher("/cilindro", Cilindro, latch=True,queue_size=1)
        self.cilindro_msg = Cilindro()
        self.espera_pub = rospy.Publisher("/espera", Espera, latch=True, queue_size=1)
        self.espera_msg = Espera()
    
    def mensaje_cilindro(self, name, cilindro):
        self.cilindro_msg.name = name
        self.cilindro_msg.cilindro = cilindro

    def mensaje_espera(self, name, espera):
        self.espera_msg.name = name
        self.espera_msg.espera = espera

    def callback_aparcado(self,data):
        if data.name == self.robobo1:
            if data.aparcado:
                print(data.name + " Aparcado")
                self.mensaje_espera(self.robobo2,False)
                self.espera_pub.publish(self.espera_msg)

        if data.name == self.robobo2 and data.aparcado:
            if data.aparcado:
                print(data.name + " Aparcado")
                self.mensaje_espera(self.robobo1, False)
                self.espera_pub.publish(self.espera_msg)

    def callback_cilindro(self,data):
        if data.name == self.robobo1 or data.name == self.robobo2:
            if data.cilindro:
                print(data.name + " Ha dejado un cilindro en la cinta")
                self.mensaje_cilindro(self.cinta, True)
                self.cilindro_pub.publish(self.cilindro_msg)

            if not data.cilindro:
                self.mensaje_cilindro(self.brazo1, True)
                self.cilindro_pub.publish(self.cilindro_msg)

        if data.name == self.cinta:
            if data.cilindro:
                self.mensaje_cilindro(self.brazo2, True)
                self.cilindro_pub.publish(self.cilindro_msg)

    def run(self):
        self.mensaje_espera(self.robobo2,False)
        self.espera_pub.publish(self.espera_msg)
        rospy.spin()



