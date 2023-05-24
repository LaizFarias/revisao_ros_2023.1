#! /usr/bin/env python3
# -*- coding:utf-8 -*-
import numpy as np
from numpy import random
from math import asin
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan,CompressedImage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from mobilenet import detect, net,CONFIDENCE, COLORS, CLASSES


""" 
Running
	roslaunch my_simulation caixas.launch
	rosrun aps4 aleatorio.py
"""

class Control():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz

		# HSV Filter
		self.color_param = {
			"magenta": {
				"lower": np.array([129,50,60],dtype=np.uint8),
				"upper": np.array([158,255,255],dtype=np.uint8)
			},
			"yellow": {
				"lower": np.array([25,50,70],dtype=np.uint8),
				"upper": np.array([35,255,255],dtype=np.uint8)
			},
			"blue": {
				"lower": np.array([112, 255, 255],dtype=np.uint8),
				"upper": np.array([115, 255, 255],dtype=np.uint8)
			},
			"green": {
				"lower": np.array([75, 255, 255],dtype=np.uint8),
				"upper": np.array([79, 255, 255],dtype=np.uint8)
			},	
		}

		self.time = 5
		self.target_time = None
		self.kernel = np.ones((5,5),np.uint8)
		self.n1 = random.randint(1,3) 
		# Subscribers
		self.bridge = CvBridge()
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
		self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
		self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size=1, buff_size = 2**24)
		#para a garra
		self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
		self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)	
		
		# Publishers
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

		self.cmd_vel_pub.publish(Twist())

		self.selected_color = None
		self.state = 1
		self.robot_state = "rotate"
		self.robot_machine = {
			"rotate": self.rotate,
			"checar": self.checar,
			"center_on_coord": self.center_on_coord,
			"para": self.para,
			"garra" : self.controla_garra,
						
		}

		self.cor_machine = {
			"aproxima": self.aproxima,
		}


		self.bixo_machine = {
			"aproxima": self.aproxima,
		}


		self.kp = 100
		self.initial_position = 0

	def odom_callback(self, data: Odometry):
		self.position = data.pose.pose.position

		if self.initial_position == 0:
			self.initial_position = self.position
		
		orientation_list = [data.pose.pose.orientation.x,
							data.pose.pose.orientation.y,
							data.pose.pose.orientation.z,
							data.pose.pose.orientation.w]

		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

		self.yaw = self.yaw % (2*np.pi)
	
	def laser_callback(self, msg: LaserScan) -> None:
		self.laser_msg = np.array(msg.ranges).round(decimals=2)
		self.laser_msg[self.laser_msg == 0] = np.inf


		self.laser_forward = np.min(list(self.laser_msg[0:5]) + list(self.laser_msg[354:359]))
		self.laser_backwards = np.min(list(self.laser_msg[175:185]))
	
	def color_segmentation(self, hsv: np.ndarray, lower_hsv: np.ndarray, upper_hsv: np.ndarray,) -> Point:
		""" 
		Use HSV mod space to segment the image and find the center of the object.

		Args:
			bgr (np.ndarray): image in BGR format
		
		Returns:
			Point: x, y and area of the object
		"""
		point = []
		mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

		contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		maior_contorno = None
		maior_contorno_area = 0
		

		for cnt in contornos:
			area = cv2.contourArea(cnt)
			if area > maior_contorno_area:
				maior_contorno = cnt
				maior_contorno_area = area

		M = cv2.moments(maior_contorno)
	
		if M["m00"] == 0:
			point_x = 0
			point_y = 0
			point = [point_x,point_y]

		else:
			point_x = int(M["m10"] / M["m00"])
			point_y = int(M["m01"] / M["m00"])
			point = [point_x,point_y]

		return point, maior_contorno_area 

		
	def image_callback(self, msg: CompressedImage) -> None:
		"""
		Callback function for the image topic
		"""
		"""
		Callback function for the image topic
		"""

		#todos os comando que tem a haver com a cor é feito no imagine_callback
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			img = cv_image.copy()
		except CvBridgeError as e:
			print(e)
		
		_, self.resultados = detect(net, img, CONFIDENCE, COLORS, CLASSES)
		
		self.w = hsv.shape[1]

		
		self.yellow, area_y = self.color_segmentation(hsv, self.color_param["yellow"]["lower"], self.color_param["yellow"]["upper"])
		self.magenta, area_m = self.color_segmentation(hsv, self.color_param["magenta"]["lower"], self.color_param["magenta"]["upper"])
		self.blue, area_b = self.color_segmentation(hsv, self.color_param["blue"]["lower"], self.color_param["blue"]["upper"])
		self.green, area_g = self.color_segmentation(hsv, self.color_param["green"]["lower"], self.color_param["green"]["upper"])

		self.cores = {1:"yellow",2:"magenta",3:"blue"}
		self.centro_cor = {1:self.yellow,2:self.magenta,3:self.blue}
		self.bixos = {1:"car",2:"bicycle",3:"horse"}

		if self.state == 1:
			if len(self.resultados)!=0:
				if self.resultados[0][0] == self.bixos[2]:
					self.selected_color = self.bixos[2]
					self.robot_state = "checar"
		elif self.state == 2:
			if self.centro_cor[2][0] != 0:
				self.selected_color = self.cores[2]
				self.robot_state = "checar"

				


	
	def rotate(self) -> None:
		"""
		Rotate the robot
		"""

		self.twist.angular.z = 0.5

		

	def checar(self) -> None:
		"""
		Stop the robot
		"""
		if self.selected_color == "magenta":
			# append magenta_machine to robot_machine
			self.robot_machine.update(self.cor_machine)
			self.robot_state = "aproxima"
		elif self.selected_color == "yellow":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.cor_machine)
			self.robot_state = "aproxima"
		elif self.selected_color == "blue":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.cor_machine)
			self.robot_state = "aproxima"
		elif self.selected_color == "car":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.cor_machine)
			self.robot_state = "aproxima"
		elif self.selected_color == "bicycle":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.cor_machine)
			self.robot_state = "aproxima"
		elif self.selected_color == "horse":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.cor_machine)
			self.robot_state = "aproxima"
		else:
			self.selected_color = None
			self.robot_state = "rotate"


	def aproxima(self) -> None:
		self.center_on_coord()

		if self.laser_forward >= 0.5:
			self.twist.linear.x = 0.2
		
		else:
			self.robot_state = "para"
		
	
	def para(self) -> None:
		"""
		Stop the robot
		"""
		self.twist = Twist()
		if self.state == 1:
			self.state = 2
			self.robot_state = "rotate"
		elif self.state == 2:
			self.robot_state = "garra"
			self.state = 3
		elif self.state == 3:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
	def center_on_coord(self):
		self.twist = Twist()
		err = 0
		
		if self.selected_color == "yellow":
			err = self.w/2 - self.yellow[0]
		elif self.selected_color == "magenta":
			err = self.w/2 - self.magenta[0]
		elif self.selected_color == "blue":
			err = self.w/2 - self.blue[0]
		elif self.selected_color == "car":
			err = self.w/2 - ((self.resultados[0][2][0] + self.resultados[0][3][0])/2)
		elif self.selected_color == "bicycle":
			err = self.w/2 - ((self.resultados[0][2][0] + self.resultados[0][3][0])/2)
		elif self.selected_color == "horse":
			err = self.w/2 - ((self.resultados[0][2][0] + self.resultados[0][3][0])/2)
		self.twist.angular.z = float(err)/self.kp

	def controla_garra(self):

		#levanta
		self.ombro.publish(1.0)
		rospy.sleep(1.0)
		#abaixa
		self.ombro.publish(-1.5)
		rospy.sleep(1.0)

		if self.state == 3: 
			self.robot_state = "para"



	def control(self):
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)
		
		self.rate.sleep()

def main():
	rospy.init_node('Aleatorio')
	control = Control()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()


