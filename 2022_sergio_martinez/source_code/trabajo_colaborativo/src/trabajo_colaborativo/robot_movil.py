from robobo_msgs.srv import MoveWheels
from robobo_msgs.srv import ResetWheels
from robobo_msgs.srv import MovePanTilt
from robobo_msgs.msg import Wheels
from robobo_msgs.msg import OrientationEuler
from robobo_msgs.msg import QrCode
from robobo_msgs.msg import QrCodeChange
from robobo_msgs.msg import SetBlobCommand
from robobo_msgs.msg import Blob
from robobo_msgs.msg import IRs
from std_msgs.msg import Int8, Int16, Int32
from trabajo_colaborativo.msg import Espera
from trabajo_colaborativo.msg import Cilindro
from trabajo_colaborativo.msg import Aparcado
import rospy
from math import pi, floor

#Final sin comunicación

class Robobo:
    def __init__(self,base):
        #Parámetros del robot
        self.espera = True
        self.base = base
        #Llamada de servicios y suscripción y publicación de topics
        self.moveWheels = rospy.ServiceProxy(self.base + "/moveWheels", MoveWheels)
        self.resetWheels = rospy.ServiceProxy(self.base + "/resetWheels", ResetWheels)
        self.movePanTilt = rospy.ServiceProxy(self.base + "/movePanTilt", MovePanTilt)
        self.bateria_base = rospy.Subscriber(self.base + "/battery/base", Int8, self.callback_battbase)
        self.bateria_phone = rospy.Subscriber(self.base + "/battery/phone", Int8, self.callback_battphone)
        self.wheels = rospy.Subscriber(self.base + "/wheels", Wheels, self.callback_wheels)
        self.orientation = rospy.Subscriber(self.base + "/orientation_euler", OrientationEuler, self.callback_or)
        self.QRstatus = rospy.Subscriber(self.base + "/qr/status", QrCode, self.callback_QR)
        self.QRchange = rospy.Subscriber(self.base + "/qr/change", QrCodeChange, self.callback_QRchange)
        self.set_blob = rospy.Publisher(self.base + "/set_blob", SetBlobCommand, latch = True, queue_size=1)
        self.espera_sub = rospy.Subscriber("/espera", Espera, self.callback_espera)
        self.color = SetBlobCommand()
        self.aparcado_pub = rospy.Publisher("/aparcado", Aparcado, latch = True, queue_size=1)
        self.aparcado_msg = Aparcado()
        self.cilindro_pub = rospy.Publisher("/cilindro", Cilindro, latch = True, queue_size=1)
        self.cilindro_msg = Cilindro()
        self.blob = rospy.Subscriber(self.base + "/blob", Blob, self.callback_blob)
        self.irs = rospy.Subscriber(self.base + "/irs", IRs, self.callback_IRs)
        #Valores de variables
        self.or_arriba = 35
        self.or_abajo = -145
        self.or_der = -50
        self.or_izq = 120
        self.speed = 1
        self.tiempo_wait = 0
        self.perimetro = pi*6
        self.margen_giro = 5
        self.tilt_QR = 85
        self.tilt_blob = 105
        self.maxQR = 290
        self.minQR = 170
        self.distQR = 850
        self.maxblob = 50
        self.minblob = 30
        self.distBlob = 8000
        self.QRaparcamiento = base
        self.distAparcar = 1000
        self.centroAparcar = 400
        #Variables booleanas
        self.bateria_baja = False
        self.mensaje_bateria = False
        self.recoge_cilindro = False
        self.avanzar = False
        self.izquierda = False
        self.derecha = False
        self.QR = False
        self.QRdetectado = False
        self.QRcentrado = False
        self.QRlisto = False
        self.blob_detectado = False
        self.blob_primer_centrado = False
        self.blob_segundo_centrado = False
        self.irs_cilindro = False
        self.cilindro_listo = False
        self.plaza_localizada = False
        self.sustitucion = False
        self.aparcado = False
        self.posicionado = False

    def mover_ruedas(self,lSpeed,rSpeed,seconds):
        #Llama al servicio para mover las ruedas con la velocidad y tiempo deseados
        self.moveWheels(Int8(lSpeed), Int8(rSpeed), Int32(seconds*1000), Int16(0)) 
    
    def reset_ruedas(self):
        #Pone los valores de los encoders de las ruedas (posición,velocidad) a 0
        self.resetWheels()

    def mover_pantilt(self,panPos,panSpeed,tiltPos,tiltSpeed):
        #Llama al servicio para mover el pan o el tilt a la posición deseada a la velocidad deseada
        self.movePanTilt(Int16(panPos), Int8(panSpeed), Int16(1), Int16(tiltPos), Int8(tiltSpeed), Int16(1))
        self.wait(5)

    def wait(self,seconds):
        rospy.sleep(seconds)

    def set_color(self, red, blue, green, custom):
        #Definición del mensaje para que el robot vea el cilindro del color deseado.
        self.color.red = red
        self.color.blue = blue
        self.color.green = green
        self.color.custom = custom
        
    def mensaje_cilindro(self, cilindro):
        #Definición del mensaje que se publica en /cilindro
        self.cilindro_msg.name = self.base
        self.cilindro_msg.cilindro = cilindro

    def mensaje_aparcado(self):
        #Definición del mensaje que se publica en /aparcado
        self.aparcado_msg.name = self.base
        self.aparcado_msg.aparcado = True

    def callback_espera(self,data):
        #Callback del topic /espera
        if data.name == self.base:
            print(self.base + " Comienza ciclo de trabajo")
            self.espera = data.espera

    def callback_battphone(self,data):
        #Callback que recibe el porcentaje de batería del teléfono.
        if not self.espera: 
            bateria = data.data
            if bateria < 30:
                self.bateria_baja = True
                if not self.mensaje_bateria:
                    print("Batería baja del teléfono: se procede a sustitución del robot")
                    self.mensaje_bateria = True

    def callback_battbase(self,data):
        #Callback que recibe el porcentaje de batería de la base.
        if not self.espera:  
            bateria = data.data
            if bateria < 30:
                self.bateria_baja = True
                if not self.mensaje_bateria:
                    print("Batería baja de la base: se procede a sustitución del robot")
                    self.mensaje_bateria = True

    def callback_wheels(self,data):
        #Callback de los encoders de las ruedas
        if not self.espera: 
            izq = abs(data.wheelPosL.data)
            der = abs(data.wheelPosR.data)
        
            if self.avanzar:
                if izq < self.grados_avanzar or der < self.grados_avanzar:
                    self.mover_ruedas(self.speed,self.speed,1)
                else:
                    self.mover_ruedas(0,0,1)
                    self.avanzar = False

    def callback_or(self,data):
        #Callback del giroscopio
        if not self.espera: 
            yaw = floor(data.yaw.data)
            if self.izquierda:
                if yaw > (self.giro+self.margen_giro) or yaw < (self.giro-self.margen_giro):
                    self.mover_ruedas(0,self.speed,1)
                else:
                    self.mover_ruedas(0,0,1)
                    self.izquierda = False
            if self.derecha:
                if yaw > self.giro+3 or yaw < self.giro-3:
                    self.mover_ruedas(self.speed,0,1)
                else:
                    self.mover_ruedas(0,0,1)
                    self.derecha = False

    def callback_QR(self,data):
        #Callback de la detección de códigos QR
        if self.QR and not self.espera:
            self.QRdetectado = True
            distancia = data.distance
            centro = data.center.x
            self.QRname = data.text
            if self.QRname == self.QRaparcamiento:
                self.aparcar_QR(centro,distancia)
            else:
                self.centradoQR(centro,distancia)

    def callback_QRchange(self,data):
        #Recibe cuando se ve y deja de ver un QR. Cuando se deja de ver, ordena al robot continuar en linea recta para que no se pare, con la esperanza de ver
        #el QR desde más cerca.
        if self.QR and not self.espera:
            if not self.QRlisto and data.distance == 0.0:
                self.QRdetectado = False
                print("He dejado de ver el QR")
                while not self.QRdetectado:
                    self.mover_ruedas(1,1,1)

    def callback_blob(self,data):
        #Callback de la detección de color
        if self.recoge_cilindro and not self.espera:
            self.blob_detectado = True
            centro = data.center.x
            distancia = data.size.data
            print(distancia)
            self.recogida_blob(centro,distancia)

    def callback_IRs(self,data):
        #Callback de los infrarrojos
        if self.irs_cilindro and not self.espera:
            print(data.FrontC.range)
            if data.FrontC.range > 1500:
                self.mover_ruedas(0,0,1)
                self.cilindro_listo = True
                self.irs_cilindro = False



    def recogida_blob(self,centro,distancia):
        #Se encarga de centrar el robot con el QR cuando sea necesario. 
        if not self.blob_primer_centrado:
            self.centrado_blob(centro)
            print("Primer centrado")
        else:
            if distancia < self.distBlob:
                print("Primer avance")
                self.mover_ruedas(self.speed,self.speed,1)
            else:
                if not self.blob_segundo_centrado:
                    print("Segundo centrado")
                    self.centrado_blob(centro)
                else:
                    self.irs_cilindro = True
                    if not self.cilindro_listo:
                        print("segundo avance")
                        self.mover_ruedas(self.speed,self.speed,1)

    def centrado_blob(self,centro):
        #Parte del centrado con respecto al color del recogida_blob
        if centro > self.maxblob:
                self.mover_ruedas(0,self.speed,1)
        elif centro < self.minblob:
            self.mover_ruedas(self.speed,0,1)
        else:
            self.mover_ruedas(0,0,1)
            if not self.blob_primer_centrado:
                self.blob_primer_centrado = True
            else:
                self.blob_segundo_centrado = True

            
    def centradoQR(self,centro,distancia):
        #Se encarga de centrar el robot con el QR cuando sea necesario. 
        if not self.QRcentrado:
            if centro > self.maxQR:
                self.mover_ruedas(0,self.speed,1)
            elif centro < self.minQR:
                self.mover_ruedas(self.speed,0,1)
            else:
                self.QRcentrado = True
        else:
            if distancia < self.distQR:
                self.mover_ruedas(self.speed,self.speed,1)
            else:
                self.mover_ruedas(0,0,0)
                self.QRlisto = True

    def aparcar_QR(self,centro,distancia):
        #Función que controla el aparcamiento del robot al quedarse sin batería
        if not self.posicionado:
            if centro > self.centroAparcar:
                print("centrando...")
                self.mover_ruedas(self.speed,self.speed,1)
            else:
                print("centrado!")
                self.plaza_localizada = True
        else:
            if distancia < self.distAparcar:
                print("Aparcando...")
                self.mover_ruedas(self.speed,self.speed,1)
            else:
                print("Aparcado")
                self.mover_ruedas(0,0,1)
                self.aparcado = True

    def avanzar_distancia(self, distancia):
        #Función para avanzar una distancia determinada con los encoders de las ruedas
        self.reset_ruedas()
        self.grados_avanzar = round(360/self.perimetro * distancia)
        self.avanzar = True

        self.mover_ruedas(self.speed,self.speed,1)
        while self.avanzar:
            self.wait(self.tiempo_wait)

        self.wait(1)

    def girar_derecha(self,orientacion):
        #Función para girar a la derecha a una determinada orientación
        self.giro = orientacion
        self.derecha = True
        while self.derecha:
            self.wait(self.tiempo_wait)

    def girar_izquierda(self,orientacion):
        #Función para girar a la izquierda a una determinada orientación
        self.giro = orientacion
        self.izquierda = True
        while self.izquierda:
            self.wait(self.tiempo_wait)

    def salida_aparcamiento(self):
        #Acción de salir de la Zona de recarga
        self.speed = -self.speed
        self.avanzar_distancia(45)
        self.speed = - self.speed
        self.girar_izquierda(self.or_der)

    def avanzar_QR(self):
        #Acciones para el avance y centrado con el QR
        self.QR = True 
        while not self.QRdetectado:
            self.mover_ruedas(self.speed,self.speed,1)
        print("QR visto")
        while not self.QRlisto:
            self.wait(self.tiempo_wait)
        print("QR listo")
        self.QR = False

    def accion_QR(self):
        #Actuaciones dependiendo del QR visto.

        #Primer caso: entrada en la zona de recogida del cilndro
        if self.QRname == "almacen":
            #Se indica que no se ha recogido el cilindro a la entrada de la zona de recogida para que funcione el callback del color.
            #self.reinicio_variables_blob()
            self.reinicio_variables_cilindro()
            self.girar_derecha(self.or_abajo)
            self.avanzar_distancia(10)
            self.girar_izquierda(self.or_der)
            self.wait(1)
        #Segundo caso: dejar el cilindro al brazo. Primero deja el cilindro y vuelve a la posición inicial. Visto el QR otra vez, gira a la izquierda.
        if self.QRname == "dejar cilindro":
            self.girar_derecha(self.or_arriba)
            self.avanzar_distancia(20)
            self.mensaje_cilindro(True)
            self.cilindro_pub.publish(self.cilindro_msg)
            self.speed = -self.speed
            self.avanzar_distancia(35)
            self.speed = - self.speed
            self.girar_izquierda(self.or_izq)
            self.speed = -self.speed
            self.avanzar_distancia(10)
            self.speed = - self.speed
            self.girar_izquierda(self.or_abajo)
        #Tercer caso: garaje. Si la variable de baja batería no es True, simplemente gira a la izquierda. En caso contrario, inicia los procesos para aparcar y
        #ser sustituido.
        if self.QRname == "aparcamiento":
            if not self.bateria_baja:
                self.girar_izquierda(self.or_der)
            else:
                self.girar_izquierda(self.or_der)
                self.mover_pantilt(270,15,0,0)
                self.sustitucion = True

        #Cuarto y quinto caso: Girar a la izquierda
        if self.QRname == "velocidad 50":
            self.girar_izquierda(self.or_izq)

        if self.QRname == "parar":
            self.girar_izquierda(self.or_arriba)

    def recoger_cilindro(self):
        #Acciones para recoger el cilindro en la Zona de recogida
        print("Buscar color")
        self.mover_pantilt(0,0,self.tilt_blob,15)
        self.recoge_cilindro = True
        while not self.blob_detectado:
            self.mover_ruedas(self.speed,self.speed,1)
        print("Color visto")
        while not self.cilindro_listo:
            self.wait(self.tiempo_wait)
        self.recoge_cilindro = False
        self.mover_pantilt(0,0,self.tilt_QR,15)

    def aparcar(self):
        #Acciones para aparcar el robot cuando se queda sin batería. Se complementa con aparcar_QR
        self.QR = True
        while not self.plaza_localizada:
            self.mover_ruedas(self.speed,self.speed,1)
        self.mover_ruedas(0,0,1)
        self.QR = False
        self.mover_pantilt(180,15,0,0)
        self.girar_derecha(self.or_abajo)
        print("posicionado")
        self.posicionado = True
        self.QR = True
        self.mover_ruedas(self.speed,self.speed,1)
        while not self.aparcado:
            self.wait(self.tiempo_wait)
        self.QR = False
        self.mover_pantilt(0,0,75,15)
        print("Final")
        self.espera = True

    def reinicio_variables_QR(self):
        #Reinicia las variables para comenzar nuevamente con el ciclo
        self.QRdetectado = False
        self.QRcentrado = False
        self.QRlisto = False

    def reinicio_variables_cilindro(self):
        #Reincio de las variables de la recogida del cilindro
        self.blob_detectado = False
        self.blob_primer_centrado = False
        self.blob_segundo_centrado = False
        self.irs_cilindro = False
        self.cilindro_listo = False
        
    def reinicio_variables_aparcar(self):
        #Reinicio de las variables del estacionamiento
        self.QRdetectado = False
        self.plaza_localizada = False
        self.sustitucion = False
        self.aparcado = False
        self.posicionado = False
        

    def run(self):
        #Ejecución principal
        while True:
            #Espera
            while self.espera:
                self.wait(self.tiempo_wait)
            #Pasos inciciales
            self.set_color(False,True,False,False)
            self.set_blob.publish(self.color)
            self.mover_pantilt(0,0,self.tilt_QR,15)
            self.salida_aparcamiento()
            while not self.sustitucion:
                #Ciclo de trabajo
                self.avanzar_QR()
                self.accion_QR()
                self.reinicio_variables_QR()
                if not self.cilindro_listo:
                    self.recoger_cilindro()
                    self.mensaje_cilindro(False)
                    self.cilindro_pub.publish(self.cilindro_msg)
            #Aparcamiento
            self.aparcar()
            self.reinicio_variables_aparcar()
            self.mensaje_aparcado()
            self.aparcado_pub.publish(self.aparcado_msg)