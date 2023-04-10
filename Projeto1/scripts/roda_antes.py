#!/usr/bin/env python3
# -*- coding:utf-8 -*-


#   exemplo adaptado do livro:
#   
#  Programming Robots with ROS.
#  A Practical Introduction to the Robot Operating System
#  Example 12-5. follower_p.py pag265
#  
#  Referendia PD:https://github.com/martinohanlon/RobotPID/blob/master/mock/mock_robot_pd.py 

from pyrsistent import s
import rospy
import os
import sys
import numpy as np
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
#import cv2.aruco as aruco
#from aruco import roda_todo_frame

class Follower:

    def __init__(self):

        
        self.bridge = CvBridge()
        self.cv_image = None
	    #topico da camera do robo real
	    #self.image_sub = rospy.Subscriber('/v4l/camera/image_raw/compressed',
        #topico da camera do robo simulado
        self.image_sub = rospy.Subscriber('/camera/image/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                             Twist, 
                                             queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                  LaserScan, 
	 		                                    self.laser_callback)

        self.odom_subscriber = rospy.Subscriber('/odom',
                                                  Odometry, 
	 		                                    self.odom_callback)
        
        self.twist = Twist()
        self.laser_msg = LaserScan()
        self.odom_msg = Odometry()
        self.isFirst = True
        self.time_ini = rospy.Time.now()
        self.odom_ini = Odometry()
        
        self.cx = -1
        self.cx_red = -1
        self.cx_blue = -1

        self.cy = -1
        self.cy_red = -1
        self.cy_blue = -1

        self.size_blue = [0,None]
        self.size_red = [0,None]

        self.h = -1
        self.w = -1

#     #     self.lastError = 0
#     #     self.max_vel_linear = 0.2
#     #     self.max_vel_angular = 2.0
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)
        self.estado = 0
        self.val = 0

    def laser_callback(self, msg):
         self.laser_msg = msg

#     # def get_laser(self, pos):
#     #     return self.laser_msg.ranges[pos]

    def odom_callback(self, msg):
        self.odom_msg = msg
        if self.isFirst and (rospy.Time.now() - self.time_ini > rospy.Duration(10)):
            self.odom_ini = self.odom_msg
            self.isFirst = False
    
    def odom_distance(self):
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y
        x0 = self.odom_ini.pose.pose.position.x
        y0 = self.odom_ini.pose.pose.position.y
        return math.sqrt((x-x0)**2 + (y-y0)**2)

    
    def image_callback(self, msg):
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
	    #cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
	
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 150, 150],dtype=np.uint8)
            upper_yellow = np.array([50, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            def percorreArea(lista):
                area_max = 0
                contorno = None
                for el in lista:
                    area = cv2.contourArea(el)
                    if area > area_max:
                        area_max = area
                        contorno = el
                output = [area_max,contorno]
                return output

            lower_blue = np.array([110, 150, 150],dtype=np.uint8)
            upper_blue = np.array([130, 255, 255],dtype=np.uint8)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            contours_blue,tree_b = cv2.findContours(mask_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            self.size_blue = percorreArea(contours_blue)
            
            lower_pink = np.array([150, 150, 150],dtype=np.uint8)
            upper_pink = np.array([170, 255, 255],dtype=np.uint8)
            mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)

            lower_green = np.array([50, 150, 150],dtype=np.uint8)
            upper_green = np.array([70, 255, 255],dtype=np.uint8)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            lower_red = np.array([0, 150, 150],dtype=np.uint8)
            upper_red = np.array([10, 255, 255],dtype=np.uint8)
            mask_red = cv2.inRange(hsv, lower_red, upper_red)

            lower_red2 = np.array([170, 50, 50],dtype=np.uint8)
            upper_red2 = np.array([180, 255, 255],dtype=np.uint8)
            mask_red += cv2.inRange(hsv, lower_red2, upper_red2)
            contours_red,tree_r = cv2.findContours(mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            self.size_red = percorreArea(contours_red)

            h, w, d = cv_image.shape
            search_top = 3*h//4
            search_bot = 3*h//4 + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            self.w = w
            self.h = h

            M = cv2.moments(mask)
            M_red = cv2.moments(mask_red)
            M_blue = cv2.moments(mask_blue)

            if self.estado == 0:

                if M['m00'] > 0:
                    self.cx = int(M['m10']/M['m00'])
                    self.cy = int(M['m01']/M['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)

            if self.estado == 1:

                if M_red['m00'] > 0:
                    self.cx_red = int(M_red['m10']/M_red['m00'])
                    self.cy_red = int(M_red['m01']/M_red['m00'])
                    cv2.circle(cv_image, (self.cx_red, self.cy_red), 20, (0,0,255), -1)

                if M_blue['m00'] > 0:
                    self.cx_blue = int(M_blue['m10']/M_blue['m00'])
                    self.cy_blue = int(M_blue['m01']/M_blue['m00'])
                    cv2.circle(cv_image, (self.cx_blue, self.cy_blue), 20, (255,0,0), -1)

            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)
    
    def segue_linha(self):
        err = self.cx - self.w/2
        #------controle P simples--------------------
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        if self.size_blue[0] > 50000 or self.size_red[0] > 50000:
            return "Slalom"
        elif self.odom_distance() < 0.8:
            return "Procurando creeper"
        return "Seguindo linha"

    def slalom(self):
        if self.size_blue[0] > self.size_red[0]:
            cx = self.cx_blue
            print("Centro Azul")
        else:
            cx = self.cx_red
            print("Centro Vermelho")
        err = cx- self.w/2
        #------controle P simples--------------------
        self.twist.linear.x = 0.15

        def calcula_coiso(err, valor):

            if valor == 0:
                self.twist.angular.z = float(err)/200
                self.twist.linear.x = 0
                if abs(err) < 100:
                    self.val += 1
                    self.time_ini = rospy.Time.now()

            if valor == 1:    
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0
                if rospy.Time.now() - self.time_ini > rospy.Duration(2):
                    self.val += 1
                    self.time_ini = rospy.Time.now()


            if valor == 2:    
                self.twist.angular.z = -float(err)/200
                self.twist.linear.x = 0
                if rospy.Time.now() - self.time_ini > rospy.Duration(0.5):
                    self.val = 0

        calcula_coiso(err, self.val)
        print(self.val)
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.5)
        if self.size_blue[0] < 50000 and self.size_red[0] < 50000:
            return "Seguindo linha"
        elif self.odom_distance() < 0.8:
            return "Procurando creeper"
        return "Slalom"

    def procura_creeper(self):
        return "Procurando creeper"

    
    def control(self):
        ### BEGIN CONTROL
        #print(self.estado)
        if self.estado == 0:
            token = self.segue_linha()
            if token == "Slalom":
                self.estado += 1
            #elif token == "Procurando creeper":
                #self.estado += 2
        
        if self.estado == 1:
            token = self.slalom()
            if token == "Seguindo linha":
                self.estado -= 1
            #elif token == "Procurando creeper":
                #self.estado += 1


        if self.estado == 2:
            token = self.procura_creeper()
            if token == "Creeper encontrado":
                self.estado += 1

        ### END CONTROL
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()

# Main loop
if __name__=="__main__":
    rospy.init_node('follower')
    follower = Follower()

    while not rospy.is_shutdown():
        follower.control()

# END ALL
