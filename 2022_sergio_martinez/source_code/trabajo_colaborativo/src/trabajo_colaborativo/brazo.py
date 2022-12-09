import rospy
from trabajo_colaborativo.msg import Espera
from trabajo_colaborativo.msg import Cilindro
from trabajo_colaborativo.msg import Aparcado

class Brazo:
    def __init__(self,brazo_name):
        self.brazo_name = brazo_name
        self.cilindro_sub = rospy.Subscriber("/cilindro", Cilindro, self.callback_cilindro)

    def callback_cilindro(self,data):
        #Callback de los mensajes que se publican en /cilidro
        if data.name == self.brazo_name:
            print(data.name + " procede a recoger nuevo cilindro")

    def run(self):
        rospy.spin()