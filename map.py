import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from math import radians
import math
import random
from sensor_msgs.msg import LaserScan #Para el laser-Scan.
from std_msgs.msg import String #Modo de entrada.
from geometry_msgs.msg import Twist

class Mapeo(object):

    def __init__(self):

        self.phi=0    #una variable con memoria de theta
        self.Tx=0
        self.Ty=0
        global Ts
        Ts = 0.01

        self.estado_anterior=' '
        rospy.loginfo("Starting node")
        #rospy.on_shutdown(self.safety_stop)
        #self.pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist)
        rospy.Subscriber('/odom', Odometry, self.sendPos)
        rospy.Subscriber('/scan',LaserScan,self.function)
        #rospy.Subscriber('/Pos',NodoPos,self.function)
        rospy.spin()

    def sendPos(self,data):
      #  X = data.x
#        print dir(data)
#        print (data.pose)
#        print (data.twist)
        x = (data.twist.twist.linear.x) # Velocidad lineal en x
#        print (X)
        W = (data.twist.twist.angular.z) # Velocidad angular
#        print (W)
        self.phi=self.phi+Ts*W
        vx=x*math.cos(self.phi)
        vy=x*math.sin(self.phi)
        self.Tx=self.Tx+vx*Ts
        self.Ty=self.Ty+vy*Ts
        #print ('%.3f' % (self.Tx))
        #print ('%.3f' % (self.Ty))


    def function(self,data):
        datos_laser = np.asarray(data.ranges)
        self.angle_min = data.angle_min*180.0/np.pi
        self.angle_max = data.angle_max*180.0/np.pi
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
        self.xi = np.sin(-self.tethas)
        self.xi = self.xi*datos_laser
        self.yi= np.cos(self.tethas)
        self.yi = self.yi*datos_laser
        #plt.plot(self.xi,self.yi)
        #plt.show()
        self.a = np.array([[1,0,self.Tx], [0,1,self.Ty], [0,0,1]]) #Matriz Traslacional
        self.b = np.array([[self.xi],[self.yi],[1]])
        self.c = np.multiply(self.a, self.b)
        #print(c)

        self.d = np.array([[math.cos(self.phi),-math.sin(self.phi),0], [math.sin(self.phi),math.cos(self.phi),0], [0,0,1]])
        self.e = (self.d*self.a*self.b)
        #f = np.multiply(e, b)
        print(self.e)

        plt.plot(self.xi,self.yi)
        plt.show()
if __name__ == '__main__':
    rospy.init_node("Mapeo")
    cv = Mapeo()
