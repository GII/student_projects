#! /usr/bin/env python3
from ctypes import py_object
from ctypes.wintypes import PINT
from pdb import Restart
import re
import sys
import copy
from turtle import position
from types import coroutine
import rospy
import tf2_ros
import moveit_commander
import geometry_msgs.msg
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from onrobot_2fg7_msgs.msg import OnRobot2FG7Output
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import PlanningScene, ObjectColor
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from math import pi
from std_msgs.msg import String
from matplotlib import pyplot as plt
import numpy as np 
import os

objeto_x=None
objeto_y=None
objeto_z=None
REFERENCE_FRAME = "base_link"
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")

scene_pub = ""


scene_pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)

# Allow some leeway in position (meters) and orientation (radians)
arm_group.set_goal_position_tolerance(0.01)
arm_group.set_goal_orientation_tolerance(0.01)

# Show replanning to increase the odds of a solution
arm_group.allow_replanning(True)
arm_group.set_max_velocity_scaling_factor(1) #modifico velocidad valores entre 0 y 1
arm_group.set_max_acceleration_scaling_factor(0.9) #modifico acelarcion valores entre 0 y 
arm_group.set_pose_reference_frame(REFERENCE_FRAME)  

# Show 5 seconds per planning attempt
arm_group.set_planning_time(5)

     
    
def funcion (data): 
    global objeto_x, objeto_y, objeto_z
    objeto_x=data.pose.position.x
    objeto_y=data.pose.position.y
    objeto_z=data.pose.position.z
    
    


def main():
    global scene_pub, scene, objeto_x, objeto_y, objeto_z
    rospy.init_node("grasp_detected_object", anonymous=True)
    waypoints = [] #inicio la lista waypoint
    y_real = []
    x_real = []
    posicion_brazo_x=[]
    posicion_brazo_y = []


    sub=rospy.Subscriber("/mocap_node/pelota/pose", PoseStamped, funcion) #duda como puedo hacer que me devuelva cada x segundo y no continuo, y no se para que sirve la funcion
    ite=1 #solo entra en x1 algo es mal en el codigo
    rospy.sleep (1)

    x2=-objeto_y
    x2_filtro = round (x2, 2)
    y2=objeto_x
    x1=0.7

    x_actual=-objeto_y    

    wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo

    objeto_basez=objeto_z #Posicion a la que quiero llegar en z 
  

    corx_mano= (arm_group.get_current_pose().pose.position.x) #guarda la coordenada x del brazo
    cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada y del brazo 
    corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada z del brazo
    

    objeto_basex=0.35 #Posicion a la que quiero llegar en x
    objeto_basey=0 #Posicion a la que quiero llegar en y
    posex= -corx_mano+objeto_basex #Movimiento que tiene que hacer la mano en x
    posey=-cory_mano+objeto_basey #Movimiento que tiene que hacer la mano en y
    posexf=round(posex,2)

    posez= -corz_mano+objeto_basez #Movimiento que tiene que hacer la mano en z 
   
    posezf=round(posez,2)
    scale=1

    wpose.position.x += scale*posexf #introducimos el movimiento x del brazo
    wpose.position.y += scale*posey #introducimos el movimiento y del brazo
    wpose.position.z += scale*posezf #introducimos el movimiento z del brazo
    waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = arm_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    arm_group.execute(plan, wait=True)#mueve el brazo

    waypoints.clear()#limpia la lista waypoints

    wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
    posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x) #anado a la lista la posicion x de la mano 
    posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y) #anado a la lista la posicion y de la mano 

   
    while x_actual > corx_mano+0.25: #condicional mientras que la pelota no llega a la distancia 0,10 de la mano sigue entrando
        
        if ite % 2 == 0: #si ite es par entra y guarda posiciones 
            rospy.sleep(0.1)
            x2=-objeto_y
            y2=objeto_x
            objeto_basez=objeto_z
            x2_filtro = round (x2, 2)
            ite=ite+1
            y_real.append(y2)
            x_real.append(x2)

        else: #si ite es impar entra y guarda posiciones 
            rospy.sleep(0.1)
            x1=-objeto_y
            y1=objeto_x
            objeto_basez=objeto_z
            x1_filtro = round (x2, 2)
            ite=ite+1
            y_real.append(y1)
            x_real.append(x1)
   
        x_actual=-objeto_y #guarda siempre la posicion en x del objeto     

        if x2>x1:
            if y2!=y1:#se tiene que hacer este if porque depende de si la posicion x2 es la mas alejada o la mas cercana.
                m=(x2-x1)/(y2-y1) #calculo la pendiente de la mano
                y=((0.4-x1+m*y1)/m)-0.01  #predice la posicipn y en la que acabara
                v=x2-x1 #calculo la diferencia de la posicion en x
                

    
               
        if x1>x2:
            if y2!=y1:
                m=(x1-x2)/(y1-y2) #calculo la pendiente
                y=((0.4-x2+m*y2)/m)-0.01 #predice la posicion y en la que acabara.
                v=x1-x2 #calculo la diferencia de la posicion en x
                
        
        if y2 == y1 : #si se mueve en perpendicular a la zona de recogida entra en este bucle
            y2=y
            
       

        if (x2_filtro!=x1_filtro): #si esta parado no debe entrar
            if v>0.035: #filtro bajo para la velocidad
                if v<0.045: #filtro alta velocidad   
                    if x_actual<1.65:#filtro para que no empiexe 
                        if y<0.75: #dato analitico fin de mesa
                            if y>-0.75: #dato analitico fin de mesa
                                if ite>2:#se realiza el if porque sino queda fuera del alcance
                                    while x_actual > corx_mano+0.25:
                                        print ("entro a  veloidad alta")
                                        x_actual=-objeto_y
                                        if ite % 2 == 0:
                                            rospy.sleep(0.1)
                                            x2=-objeto_y
                                            y2=objeto_x
                                            objeto_basez=objeto_z
                                            x2_filtro = round (x2, 2)
                                            ite=ite+1 
                                            y_real.append(y2)
                                            x_real.append(x2)
                                        else:
                                            rospy.sleep(0.1)
                                            x1=-objeto_y
                                            y1=objeto_x
                                            objeto_basez=objeto_z
                                            x1_filtro = round (x2, 2)
                                            ite=ite+1 
                                            y_real.append(y1)
                                            x_real.append(x1)


                                        if x2>x1: 
                                            if y2!=y1:#se tiene que hacer este if porque depende de si la posicion x2 es la mas alejada o la mas cercana.
                                                m=(x2-x1)/(y2-y1) #calculo la pendiente de la mano
                                                y=((0.40-x1+m*y1))/m-0.01 #predice la posicipn y en la que acabara
                                               

                        
                                        if x1>x2: 
                                            if y2!=y1:
                                                m=(x1-x2)/(y1-y2) #calculo la pendiente
                                                y=((0.40-x2+m*y2)/m)-0.01 #predice la posicion y en la que acabara.
                                            

                                        if y2 == y1 :
                                            y2=y
                                            
                                           
                                        
                                        wpose = arm_group.get_current_pose().pose
                                        corx_mano= (arm_group.get_current_pose().pose.position.x)
                                        cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                                        corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                                        #posicion del objeto respecto a la base
                                
                                        objeto_basey=y #NEGATIVO HACIA LA IZQUIERDA DEL ROBOT Y POSITIVO HACIA LA DERECHA DEL ROBOT 
                                        objeto_basez=objeto_z 
                            

                                        posey=cory_mano-(objeto_basey-0.020)
                                        posez=-corz_mano+objeto_basez
                            
                                        poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                                        posezf=round(posez,2)
                            

                                        wpose = arm_group.get_current_pose().pose #guarda arm.group... donde luego se anadira la zona a la que se mover
                        
                                        scale = 1 

                                        wpose.position.y -= scale*poseyf  # Third move sideways (y)
                                        if posezf !=0:
                                            wpose.position.z += scale*posezf
                                        waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                                        # We want the Cartesian path to be interpolated at a resolution of 1 cm
                                        # which is why we will specify 0.01 as the eef_step in Cartesian
                                        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
                                        (plan, fraction) = arm_group.compute_cartesian_path(
                                                                        waypoints,   # waypoints to follow
                                                                        0.01,        # eef_step
                                                                        0.0)         # jump_threshold

                                        # Note: We are just planning, not asking move_group to actually move the robot yet:
                    
                                        arm_group.execute(plan, wait=True)#mueve el brazo
                                        waypoints.clear()#limpia la lista waypoints
                                        wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                        posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                        posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y)
 
        if (x2_filtro!=x1_filtro):
            if v>0.02: #filtro bajo para la velocidad
                if v<0.035: #filtro alta velocidad   
                    if x_actual<1.2:#filtro para que no empiece 
                        if y<0.75: #dato analitico fin de mesa
                            if y>-0.75: #dato analitico fin de mesa
                                if ite>2:#se realiza el if porque sino queda fuera del alcance.
                                    print ("entro a velocidad media")
                                    while x_actual > corx_mano+0.25:
                                        x_actual=-objeto_y
                                        if ite % 2 == 0:
                                            rospy.sleep(0.1)
                                            x2=-objeto_y
                                            y2=objeto_x
                                            objeto_basez=objeto_z
                                            x2_filtro = round (x2, 2)
                                            ite=ite+1 
                                            y_real.append(y2)
                                            x_real.append(x2)
                                        else:
                                            rospy.sleep(0.1)
                                            x1=-objeto_y
                                            y1=objeto_x
                                            objeto_basez=objeto_z
                                            x1_filtro = round (x2, 2)
                                            ite=ite+1 
                                            y_real.append(y1)
                                            x_real.append(x1)

                                        if x2>x1: 
                                            if y2!=y1:#se tiene que hacer este if porque depende de si la posicion x2 es la mas alejada o la mas cercana.
                                                m=(x2-x1)/(y2-y1) #calculo la pendiente de la mano
                                                y=((0.40-x1+m*y1)/m)-0.01  #predice la posicipn y en la que acabara
                                              

                        
                                        if x1>x2: 
                                            if y2!=y1:
                                                m=(x1-x2)/(y1-y2) #calculo la pendiente
                                                y=((0.40-x2+m*y2)/m)-0.018 #predice la posicion y en la que acabara.
                                           

                                        if y2 == y1 :
                                            y2=y
                                            
                                           
                                        
                                        wpose = arm_group.get_current_pose().pose
                                        corx_mano= (arm_group.get_current_pose().pose.position.x)
                                        cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                                        corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                                        #posicion del objeto respecto a la base
                                
                                        objeto_basey=y #NEGATIVO HACIA LA IZQUIERDA DEL ROBOT Y POSITIVO HACIA LA DERECHA DEL ROBOT 
                                        objeto_basez=objeto_z 
                            

                                        posey=cory_mano-(objeto_basey-0.020)
                                        posez=-corz_mano+objeto_basez
                            
                                        poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                                        posezf=round(posez,2) 
                            

                                        wpose = arm_group.get_current_pose().pose #guarda arm.group... donde luego se anadira la zona a la que se mover
                        
                                        scale = 1 #escala para que este en metros

                                
                                        #waypoints.append(copy.deepcopy(wpose))
                                        wpose.position.y -= scale*poseyf  # Third move sideways (y)
                                        if posezf >0.02:
                                            wpose.position.z += scale*posezf
                                        waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                                        # We want the Cartesian path to be interpolated at a resolution of 1 cm
                                        # which is why we will specify 0.01 as the eef_step in Cartesian
                                        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
                                        (plan, fraction) = arm_group.compute_cartesian_path(
                                                                        waypoints,   # waypoints to follow
                                                                        0.01,        # eef_step
                                                                        0.0)         # jump_threshold

                                        # Note: We are just planning, not asking move_group to actually move the robot yet:
                    
                                        arm_group.execute(plan, wait=True)#mueve el brazo
                                        waypoints.clear()#limpia la lista waypoints
                                        wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                        posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                        posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y)  
        if (x2_filtro!=x1_filtro):
            if v<0.02: #filtro alta velocidad   
                if x_actual<0.8:#filtro para que no empiece 
                    if y<0.75: #dato analitico fin de mesa
                        if y>-0.75: #dato analitico fin de mesa
                            if ite>2:#se realiza el if porque sino queda fuera del alcance
                                while x_actual > corx_mano+0.25:
                                    x_actual=-objeto_y
                                    print ("entro a baja")
                                    if ite % 2 == 0:
                                        rospy.sleep(0.1)
                                        x2=-objeto_y
                                        y2=objeto_x
                                        objeto_basez=objeto_z
                                        x2_filtro = round (x2, 2)
                                        ite=ite+1 
                                        y_real.append(y2)
                                        x_real.append(x2)
                                    else:
                                        rospy.sleep(0.1)
                                        x1=-objeto_y
                                        y1=objeto_x
                                        objeto_basez=objeto_z
                                        x1_filtro = round (x2, 2)
                                        ite=ite+1 
                                        y_real.append(y1)
                                        x_real.append(x1)

                                    if x2>x1: 
                                        if y2!=y1:#se tiene que hacer este if porque depende de si la posicion x2 es la mas alejada o la mas cercana.
                                            m=(x2-x1)/(y2-y1) #calculo la pendiente de la mano
                                            y=((0.40-x1+m*y1)/m)-0.01  #predice la posicipn y en la que acabara
                                      

                        
                                    if x1>x2: 
                                        if y2!=y1:
                                            m=(x1-x2)/(y1-y2) #calculo la pendiente
                                            y=((0.40-x2+m*y2)/m)-0.01 #predice la posicion y en la que acabara.
                                

                                    if y2 == y1 :
                                        y2=y
                                            
                                    wpose = arm_group.get_current_pose().pose
                                    cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                                    corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                                    #posicion del objeto respecto a la base
                                
                                    objeto_basey=y #NEGATIVO HACIA LA IZQUIERDA DEL ROBOT Y POSITIVO HACIA LA DERECHA DEL ROBOT 
                                    objeto_basez=objeto_z 
                            

                                    posey=cory_mano-(objeto_basey-0.020)
                                    posez=-corz_mano+objeto_basez
                            
                                    poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                                    posezf=round(posez,2) 
                            

                                    wpose = arm_group.get_current_pose().pose #guarda arm.group... donde luego se anadira la zona a la que se mover
                        
                                    scale = 1 #escala para que este en metros

                                
                                    #waypoints.append(copy.deepcopy(wpose))
                                    wpose.position.y -= scale*poseyf  # Third move sideways (y)
                                    if posezf > 0.02:
                                        wpose.position.z += scale*posezf
                                    waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                                    # We want the Cartesian path to be interpolated at a resolution of 1 cm
                                    # which is why we will specify 0.01 as the eef_step in Cartesian
                                    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
                                    (plan, fraction) = arm_group.compute_cartesian_path(
                                                                    waypoints,   # waypoints to follow
                                                                    0.01,        # eef_step
                                                                    0.0)         # jump_threshold

                                    # Note: We are just planning, not asking move_group to actually move the robot yet:
                    
                                    arm_group.execute(plan, wait=True)#mueve el brazo
                                    waypoints.clear()#limpia la lista waypoints
                                    wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                    posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                    posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y)  
        if (x2_filtro!=x1_filtro):
            if v>0.045: #filtro alta velocidad   
                if x_actual<1.7:#filtro para que no empiexe 
                    if y<0.75: #dato analitico fin de mesa
                        if y>-0.75: #dato analitico fin de mesa
                            if ite>2:#se realiza el if porque sino queda fuera del alcance
                                print ("entro a corte")
                                while x_actual > 0.2:
                                    x_actual=-objeto_y       
                                    wpose = arm_group.get_current_pose().pose
                                    corx_mano= (arm_group.get_current_pose().pose.position.x)
                                    cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                                    corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                                    #posicion del objeto respecto a la base
                                    objeto_basex=0.5
                                    objeto_basey=0.0 #NEGATIVO HACIA LA IZQUIERDA DEL ROBOT Y POSITIVO HACIA LA DERECHA DEL ROBOT 
                                    objeto_basez=0.8
                            
                                    posex=-corx_mano+objeto_basex
                                    posey=cory_mano-objeto_basey
                                    posez=-corz_mano+objeto_basez
                            
                                    poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                                    posezf=round(posez,2) 
                            

                                    wpose = arm_group.get_current_pose().pose #guarda arm.group... donde luego se anadira la zona a la que se mover
                        
                                    scale = 1 #escala para que este en metros

                                
                                    #waypoints.append(copy.deepcopy(wpose))
                                    wpose.position.x += scale*posexf
                                    wpose.position.y -= scale*poseyf  # Third move sideways (y)
                                    if posezf >0.02:
                                        wpose.position.z += scale*posezf
                                    waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                                    # We want the Cartesian path to be interpolated at a resolution of 1 cm
                                    # which is why we will specify 0.01 as the eef_step in Cartesian
                                    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
                                    (plan, fraction) = arm_group.compute_cartesian_path(
                                                                    waypoints,   # waypoints to follow
                                                                    0.01,        # eef_step
                                                                    0.0)         # jump_threshold

                                    # Note: We are just planning, not asking move_group to actually move the robot yet:
                    
                                    arm_group.execute(plan, wait=True)#mueve el brazo
                                    waypoints.clear()#limpia la lista waypoints
                                    wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo 
                                    posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                    posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y)  
    
        if y<-0.75: #0.75 limite de la mesa a partir de esta y cae la pelota
            if y2!=y1:
                if x2!=x1:
                    if x_actual<1.55:
                        x_actual=-objeto_y
                        corx_mano= (arm_group.get_current_pose().pose.position.x)       
                        while x_actual > corx_mano+0.25:

                            rospy.sleep(0.5)

                            if ite % 2 == 0:
                                x2=-objeto_y
                                y2=objeto_x
                                objeto_basez=objeto_z
                                ite=ite+1
                                y_real.append(y2)
                                x_real.append(x2)
                            else:
                                rospy.sleep(0.1)
                                x1=-objeto_y
                                y1=objeto_x
                                objeto_basez=objeto_z
                                ite=ite+1
                                y_real.append(y1)
                                x_real.append(x1)
                            

                            x_actual=-objeto_x

                            if x2>x1:#se tiene que hacer este if porque depende de si la posicion x2 es la mas alejada o la mas cercana.
                                m=(x2-x1)/(y2-y1) #calculo la pendiente de la mano
                                x=(m*(-0.75-y2)+x2)+0.14
                            
                                

                            if x1>x2:
                                m=(x1-x2)/(y1-y2) #calculo la pendiente    
                                x=(m*(-0.75-y1)+x1)+0.14
                        
                                    
                            wpose = arm_group.get_current_pose().pose
                            cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                            corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                            corx_mano= (arm_group.get_current_pose().pose.position.x) 
                            
                            objeto_basex=x
                            objeto_basez=objeto_z 
                            objeto_basey=-0.75 #posicion a la que se mueve el brazo en y
                            
                            posex=-corx_mano+objeto_basex
                            posey=cory_mano-objeto_basey
                            posez=-corz_mano+objeto_basez
                            
                            poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                            posezf=round(posez,2) 
                            posexf=round(posex,2)

                            wpose = arm_group.get_current_pose().pose #guarda arm.group... donde luego se anadira la zona a la que se mover
                        
                            scale = 1 #escala para que este en metros

                              
                            if posexf<0.5:
                                
                                wpose.position.x += scale*posexf
                                wpose.position.y -= scale*poseyf 
                            
                                if posezf > 0.05:
                                    wpose.position.z += scale*posezf
                                waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                        
                                (plan, fraction) = arm_group.compute_cartesian_path(
                                                                waypoints,   # waypoints to follow
                                                                0.01,        # eef_step
                                                                0.0)         # jump_threshold

                    
                                arm_group.execute(plan, wait=True)#mueve el brazo
                            
                                waypoints.clear()#limpia la lista waypoints
                                wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y)  

                            if posexf>0.5: #NO LLEGA A LA PELOTA Y SE LEVANTA
                                wpose = arm_group.get_current_pose().pose
                                cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                                corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                                corx_mano= (arm_group.get_current_pose().pose.position.x) 
                            
                                objeto_basex=0.50
                                objeto_basez=0.8
                                objeto_basey=0.0
                            
                                posex=-corx_mano+objeto_basex
                                posey=cory_mano-objeto_basey
                                posez=-corz_mano+objeto_basez
                                poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                                posezf=round(posez,2) 
                                posexf=round(posex,2)
                                
                                wpose.position.x += scale*posexf
                                wpose.position.y -= scale*poseyf 
                            
                                if posezf > 0.05:
                                    wpose.position.z += scale*posezf
                                waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                        
                                (plan, fraction) = arm_group.compute_cartesian_path(
                                                                waypoints,   # waypoints to follow
                                                                0.01,        # eef_step
                                                                0.0)         # jump_threshold

                    
                                arm_group.execute(plan, wait=True)#mueve el brazo
                            
                                waypoints.clear()#limpia la lista waypoints
                                wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y) 
                                print ("No llego a capturarlo")

        if y>0.75: #0.75 limite de la mesa a partir de esta se cae la pelota
            if y2!=y1:
                if x2!=x1:
                    if x_actual<1.55:
                        x_actual=-objeto_y
                        corx_mano= (arm_group.get_current_pose().pose.position.x)       
                        
                        
                        while x_actual > corx_mano+0.25:
                            
                            
                            if ite % 2 == 0:
                                x2=-objeto_y
                                y2=objeto_x
                                objeto_basez=objeto_z
                                ite=ite+1
                                y_real.append(y2)
                                x_real.append(x2)
            

                            else:
                                rospy.sleep(0.1)
                                x1=-objeto_y
                                y1=objeto_x
                                objeto_basez=objeto_z
                                ite=ite+1
                                y_real.append(y1)
                                x_real.append(x1)
                            

                            x_actual=-objeto_x

                            if x2>x1:#se tiene que hacer este if porque depende de si la posicion x2 es la mas alejada o la mas cercana.
                                m=(x2-x1)/(y2-y1) #calculo la pendiente de la mano
                                x=(m*(0.75-y2)+x2)+0.14
                            
                                

                            if x1>x2:
                                m=(x1-x2)/(y1-y2) #calculo la pendiente    
                                x=(m*(0.75-y1)+x1)+0.14
                        
                                    
                            wpose = arm_group.get_current_pose().pose
                            cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                            corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                            corx_mano= (arm_group.get_current_pose().pose.position.x) 
                            
                            objeto_basex=x
                            objeto_basez=objeto_z 
                            objeto_basey=0.75 #posicion a la que se mueve el brazo en y
                            
                            posex=-corx_mano+objeto_basex
                            posey=cory_mano-objeto_basey
                            posez=-corz_mano+objeto_basez
                            
                            poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                            posezf=round(posez,2) 
                            posexf=round(posex,2)

                            wpose = arm_group.get_current_pose().pose #guarda arm.group... donde luego se anadira la zona a la que se mover
                        
                            scale = 1 #escala para que este en metros

                              
                            if posexf<0.5: #si es menor que esta posicion puede cogerla 
                                
                                wpose.position.x += scale*posexf
                                wpose.position.y -= scale*poseyf 
                            
                                if posezf > 0.05:
                                    wpose.position.z += scale*posezf
                                waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                        
                                (plan, fraction) = arm_group.compute_cartesian_path(
                                                                waypoints,   # waypoints to follow
                                                                0.01,        # eef_step
                                                                0.0)         # jump_threshold

                    
                                arm_group.execute(plan, wait=True)#mueve el brazo
                            
                                waypoints.clear()#limpia la lista waypoints
                                wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y) 

                            if posexf>0.5: #NO LLEGA A LA PELOTA PORQUE NO ESTA EN SU ALCANCE 
                               
                                
                                wpose = arm_group.get_current_pose().pose
                                cory_mano= (arm_group.get_current_pose().pose.position.y) #guarda la coordenada de la y del brazo
                                corz_mano= (arm_group.get_current_pose().pose.position.z) #guarda la coordenada de la z del brazo
                                corx_mano= (arm_group.get_current_pose().pose.position.x) 
                                
                                objeto_basex=0.50
                                objeto_basez=0.8
                                objeto_basey=0.0

                                posex=-corx_mano+objeto_basex
                                posey=cory_mano-objeto_basey
                                posez=-corz_mano+objeto_basez
                                
                                
                                poseyf=round(posey,2) #zona a donde se mueve la pinza, se le resta la posicion de la pelota a la actual de la mano para alcanzarla
                                posezf=round(posez,2) 
                                posexf=round(posex,2)
                                
                                wpose.position.x += scale*posexf
                                wpose.position.y -= scale*poseyf 
                            
                                if posezf > 0.05:
                                    wpose.position.z += scale*posezf
                                waypoints.append(copy.deepcopy(wpose))#guarda los puntos en la lista
                                
                        
                                (plan, fraction) = arm_group.compute_cartesian_path(
                                                                waypoints,   # waypoints to follow
                                                                0.01,        # eef_step
                                                                0.0)         # jump_threshold

                    
                                arm_group.execute(plan, wait=True)#mueve el brazo
                            
                                waypoints.clear()#limpia la lista waypoints
                                wpose = arm_group.get_current_pose().pose #guarda en w.pose la posicion actual del brazo
                                posicion_brazo_x.append(arm_group.get_current_pose().pose.position.x)
                                posicion_brazo_y.append(arm_group.get_current_pose().pose.position.y) 
                                print ("No llego a capturarlo")

 

    waypoints.clear()#limpia la lista waypoints
    wpose = arm_group.get_current_pose().pose 
    #Graficas
    plt.plot (x_real, y_real, label="trayectoria de la pelota", color="tab:blue", marker="o")
    plt.plot (posicion_brazo_x, posicion_brazo_y, label="movimiento de la mano", color="tab:red", marker="o")
    legend = plt.legend(loc='upper center', shadow=True, fontsize='x-large')
    plt.ylabel ("posicion y")
    plt.xlabel ("posicion x")
    plt.show ()



    rospy.spin() #evita que el nodo salga hasta que el nodo ha sido cerrado
    moveit_commander.roscpp_shutdown() #sale del moveit

    rospy.spin()


if __name__ == "__main__":
    main()
