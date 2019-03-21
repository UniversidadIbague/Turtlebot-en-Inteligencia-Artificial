#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
from math import radians
import time
import math

class CommandVelocity():

    def __init__(self):
        self.estado_anterior=' '
        rospy.loginfo("Starting node")
        rospy.on_shutdown(self.safety_stop)
        self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist)
        rospy.Subscriber('/scan', LaserScan, self.send_velocities)
        #rospy.Subscriber('/odom', Odometry, self.obtener_distancia)
        rospy.spin()


    def send_velocities(self, data):
        rospy.loginfo("reading LiDAR")
        datos_laser = np.asarray(data.ranges)
        # --- AQUÍ EMPIEZA SU CÓDIGO ---
        self.angle_min = (data.angle_min*180.0)/np.pi
        self.angle_max = (data.angle_max*180.0)/np.pi
        print("Angulo minimo", self.angle_min)
        print("Angulo maximo", sel.angle_max)
        self.N = len(data.ranges)
        self.angle_inc = (data.angle_max-data.angle_min)/self.N*180.0/np.pi

        self.r_min = np.nanmin(datos_laser)
        self.r_max = np.nanmax(datos_laser)
        self.tethas = np.linspace(data.angle_min, data.angle_max, self.N)

        self.pos_min = np.where(datos_laser==self.r_min)
        self.pos_min = self.tethas[self.pos_min]
        self.pos_min = self.pos_min*180.0/np.pi
        self.pos_max = np.where(datos_laser==self.r_max)
        self.pos_max = self.tethas[self.pos_max]
        self.pos_max = self.pos_max*180.0/np.pi
        self.med = np.nanmean(datos_laser)

        self.x = np.sin(-self.tethas)
        self.x = self.x*datos_laser #posicion x del objeto
        self.y= np.cos(self.tethas) #Posicion y del objeto
        self.y = self.y*datos_laser

        ang_min = (data.angle_min*180)/np.pi
        ang_max = (data.angle_max*180)/np.pi
        data_length = len(data.ranges)
        resolution = (ang_max-ang_min)/data_length
        #--- IMPRIMO LOS DATOS DEL ANGULO, LONGITUD DE DATOS Y RESOLUCION ---
        #print("El angulo minimo es = ", ang_min)
        #print("El angulo maximo es = ", ang_max)
        #print("El numero de datos es = ", data_length)
        #print("La resolucion es = ", resolution)
        self.control(self.x, self.y)

    def control(self, X, Y):
        self.tethas = np.linspace(-29.883638317, 30.038820215, 640)
        self.tethas = self.tethas/180.0*np.pi
        r = self.y/np.cos(self.tethas)
        min_r = np.nanmin(r)

        right = r[0:213]
        move = r[214:426]
        left = r[427:640]

        self.right = np.nanmin(right)
        self.move = np.nanmin(move)
        self.left = np.nanmin(left)
        # --- MI REFERENCIA ---
        x = 1.5

        print("Avanzar " + str(self.move))
        print("Derecha " + str(self.right))
        print("Izquierda " + str(self.left))

        if(self.move >= x and self.right >= x and self.left >= x or min_r < 0.6 and math.isnan(self.move) == True):
            self.avanzar()
            rospy.sleep(0.3)
        elif(self.move >= x and self.right >= x and self.left < x or min_r < 0.6 and math.isnan(self.right) == True):
            self.girar_derecha()
            rospy.sleep(0.2)
        elif(self.move >= x and self.right < x and self.left >= x or min_r < 0.6 and math.isnan(self.left) == True):
            self.girar_izquierda()
            rospy.sleep(0.2)
        elif(self.move >= x and self.right < x and self.left < x):
            self.atras()
            rospy.sleep(0.3)
        elif(self.move < x and self.right < x and self.left < x):
            self.pared()
            rospy.sleep(0.2)
            self.atras_derecha()
            rospy.sleep(0.2)
            self.atras_izquierda()
            rospy.sleep(0.2)

        #if(min_r < 0.6 or math.isnan(self.right) == True):
        #    self.avanzar()
        #if(min_r < 0.6 or math.isnan(self.left) == True):
        #    self.avanzar()
        #if(min_r < 0.6 or math.isnan(self.move) == True):
        #    self.avanzar()

    def avanzar(self):
        twist_msg = Twist()
        twist_msg.linear.x=1.1
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=0.0
        self.pub.publish(twist_msg)

    def girar_izquierda(self):
        twist_msg = Twist()
        twist_msg.linear.x=0.4
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=radians(45)
        self.pub.publish(twist_msg)

    def girar_derecha(self):
        twist_msg = Twist()
        twist_msg.linear.x=0.4
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=radians(-45)
        self.pub.publish(twist_msg)

    def atras(self):
        twist_msg = Twist()
        twist_msg.linear.x=-0.6
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=radians(-90)
        self.pub.publish(twist_msg)

    def pared(self):
        twist_msg = Twist()
        twist_msg.linear.x=-1.1
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=radians(-90)
        self.pub.publish(twist_msg)

    def atras_derecha(self):
        twist_msg = Twist()
        twist_msg.linear.x=-1.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=radians(45)
        self.pub.publish(twist_msg)

    def atras_izquierda(self):
        twist_msg = Twist()
        twist_msg.linear.x=-1.0
        twist_msg.linear.y=0.0
        twist_msg.linear.z=0.0
        twist_msg.angular.x=0.0
        twist_msg.angular.y=0.0
        twist_msg.angular.z=radians(-45)
        self.pub.publish(twist_msg)



    def safety_stop(self):
        for i in range(0,5):
            self.parar()
            rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
