import rospy
from trabajo_colaborativo.msg import Espera
from trabajo_colaborativo.msg import Cilindro
from trabajo_colaborativo.msg import Aparcado

class Cinta:
    def __init__(self):
        self.cinta = "cinta"
        self.cilindro_sub = rospy.Subscriber("/cilindro", Cilindro, self.callback_cilindro)
        self.cilindro_pub = rospy.Publisher("/cilindro", Cilindro, latch=True, queue_size=1)
        self.cilindro_msg = Cilindro()

    def mensaje_cilindro(self):
        #Definición del mensaje que se publica en /cilindro
        self.cilindro_msg.name = self.cinta
        self.cilindro_msg.cilindro = True
    
    def callback_cilindro(self,data):
        #Callback de los mensajes que se publican en /cilindro
        if data.name == self.cinta:
            if data.cilindro:
                rospy.sleep(5)
                self.mensaje_cilindro()
                self.cilindro_pub.publish(self.cilindro_msg)
                print(data.name + " Cilindro posicionado para recogida")

    def run(self):
        rospy.spin()