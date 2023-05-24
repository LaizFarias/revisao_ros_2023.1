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
import cv2.aruco as aruco

class Control():
    def __init__(self):
        self.rate = rospy.Rate(250) # 250 Hz

        self.color_param = {
            "pista_inicial": {
                "lower": np.array([19, 80, 184],dtype=np.uint8),
                "upper": np.array([177, 255, 255],dtype=np.uint8)
            },
            "pista_vermelha": {
                "lower": np.array([0, 10, 223],dtype=np.uint8),
                "upper": np.array([4, 255, 255],dtype=np.uint8)
            },
            "pista_verde": {
                "lower": np.array([71, 244, 155],dtype=np.uint8),
                "upper": np.array([78, 255, 255],dtype=np.uint8)
            }
        }
        self.kernel = np.ones((5,5),np.uint8)
		# Subscribers
        self.bridge = CvBridge()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size=1, buff_size = 2**24)
        #para a garra
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)	

        #camera 
        self.camera_distortion = np.loadtxt('/home/borg/catkin_ws/src/meu_projeto/scripts/cameraDistortion_realsense.txt', delimiter=',')
        self.camera_matrix = np.loadtxt('/home/borg/catkin_ws/src/meu_projeto/scripts/cameraMatrix_realsense.txt', delimiter=',')

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)



        self.cmd_vel_pub.publish(Twist())
        self.state = 1
        self.selected_mod = None
        self.robot_state = "anda"
        self.robot_machine = {
            "rotate_direita": self.rotate_direita,
            "rotate_esquerda": self.rotate_esquerda,
            "checar": self.checar,
            "center_on_coord": self.center_on_coord,
            "para": self.para,
            "aproxima_pista": self.aproxima_pista,
            "anda" : self.anda,
            "aproxima_aruco" : self.aproxima_aruco,
            "aproxima_caixa" : self.aproxima_caixa,
            "go_to_coord" : self.go_to_coord,
            
        }


        self.initial_position = 0
        self.n1 = random.randint(1,2) 
        self.kp = 200

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

    def geraAruco(self,img):
        centros = []
        distancia_aruco=[]
        # Gera mask Cinza
        grayColor = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #Gera Dicionario com Arucos
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        #Detecta Arucos e Gera variaveis
    
        cornersList, ids, _ = aruco.detectMarkers(
            grayColor, dicionarioAruco)
        if ids is not None:
            for i in range(len(ids)):
                if ids[i]<99:
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 19, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    distancia_aruco.append(np.linalg.norm(tvec))
                else: 
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 6, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    distancia_aruco.append(np.linalg.norm(tvec))
                

            for corners in cornersList:
                for corner in corners:
                    centros.append(np.mean(corner, axis=0))

        return ids, centros, distancia_aruco
        
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

        h, w, d = img.shape
        self.centro_segue = (h, 25*h//40)
        self.centro_img = (w//2, h//2)

        self.pista, area_p = self.color_segmentation(hsv, self.color_param["pista_inicial"]["lower"], self.color_param["pista_inicial"]["upper"])
        self.pista_r, area_p = self.color_segmentation(hsv, self.color_param["pista_vermelha"]["lower"], self.color_param["pista_vermelha"]["upper"])
        self.pista_g, area_p = self.color_segmentation(hsv, self.color_param["pista_verde"]["lower"], self.color_param["pista_verde"]["upper"])
        self.ids, self.centros, self.distancia_aruco = self.geraAruco(cv_image)

        self.bixos = {1:"dog",2:"horse"}
        self.centro = {1:self.pista_r,2:self.pista_g}
        self.pista_s = {1:"pista_vermelha",2:"pista_verde"}

        if self.state == 2:
            if self.pista[0] != 0:
                self.selected_mod = "pista_inicial"
                self.robot_state = "checar"
        elif self.state == 3:
            if self.ids is not None:
                if [41] in self.ids:
                    self.idx = list(self.ids).index([41])
                    self.selected_mod = "aruco"
                    self.robot_state = "aproxima_aruco"
        elif self.state == 4: 
            if len(self.resultados)!=0:
                if self.resultados[0][0] == self.bixos[self.n1]:
                    self.selected_mod = self.bixos[self.n1]
                    self.robot_state = "checar"
        elif self.state == 5:

            if self.centro[self.n1][0] != 0:
                self.selected_mod = self.pista_s[self.n1]
                self.robot_state = "checar"


        cv2.imshow("Referencia",cv_image)
        cv2.waitKey(1)


    def rotate_direita(self) -> None:
        """
        Rotate the robot
        """
        self.twist = Twist()
        self.twist.angular.z = -0.5

    def rotate_esquerda(self) -> None:
        """
        Rotate the robot
        """
        self.twist = Twist()
        self.twist.angular.z = 0.5

    def checar(self) -> None:
        if self.selected_mod == "pista_inicial":
            self.robot_state = "aproxima_pista"
        elif self.selected_mod == "aruco":
            self.robot_state = "aproxima_aruco"
        elif self.selected_mod == "dog":
            self.robot_state = "aproxima_caixa"
        elif self.selected_mod == "horse":
            self.robot_state = "aproxima_caixa"
        elif self.selected_mod == "pista_vermelha":
            self.robot_state = "aproxima_pista"
        elif self.selected_mod == "pista_verde":
            self.robot_state = "aproxima_pista"



    def anda(self) -> None:
        if self.laser_forward > 1.2:
            self.twist.linear.x = 0.2
        else:
            self.robot_state = "para"



    def aproxima_pista(self) -> None:
        self.center_on_coord()

        self.twist.linear.x = 0.2

        if self.state == 2:
            if self.pista[0] == 0:
                self.robot_state = "para"
        elif self.state == 5:
            if self.centro[self.n1][0] == 0:
                self.robot_state = "para"
                      
            

    def aproxima_caixa(self) -> None:
        self.center_on_coord()

        if self.laser_forward >= 0.5:
            self.twist.linear.x = 0.2

        else:
            self.robot_state = "para"  

    def aproxima_aruco(self) -> None:
        self.center_on_coord()
        if (self.distancia_aruco[self.idx])/100 > 1.2:
            self.twist.linear.x = 0.2
        else:
            self.robot_state = "para"

    def para(self) -> None:
        self.twist = Twist()
        if self.state == 1:
            self.state = 2
            self.robot_state = "rotate_esquerda"
        elif self.state == 2:
            self.state = 3
            self.robot_state = "rotate_direita"
        elif self.state == 3:
            self.state = 4
            if self.bixos[self.n1]  == "dog":
                self.robot_state = "rotate_esquerda"
            else:
                self.robot_state = "rotate_direita"
        elif self.state == 4:
            self.state = 5
            if self.bixos[self.n1]  == "dog":
                self.robot_state = "rotate_esquerda"
            else:
                self.robot_state = "rotate_direita"
        elif self.state == 5:
            self.robot_state = "go_to_coord"

            

    def center_on_coord(self):
        self.twist = Twist()
        err = 0

        if self.selected_mod == "pista_inicial":
            err = self.centro_segue[0] - self.pista[0]

        elif self.selected_mod == "aruco":
            err = self.centro_img[0] - self.centros[self.idx][0]

        elif self.selected_mod == "dog":
            err = self.centro_img[0] - self.resultados[0][2][0]

        elif self.selected_mod == "horse":
            err = self.centro_img[0] - self.resultados[0][2][0]

        elif self.selected_mod == "pista_vermelha":
            err = self.centro_segue[0] - self.pista_r[0]
        elif self.selected_mod == "pista_verde":
            err = self.centro_segue[0] - self.pista_g[0]


        self.twist.angular.z = float(err)/self.kp # maior o kp (mais preciso é o robo) , quanto menor menos preciso

    def go_to_coord(self):
        self.twist = Twist()
        difx = self.initial_position.x - self.position.x
        dify = self.initial_position.y - self.position.y

        angulo = abs(asin(dify/((difx**2 + dify**2)**0.5)))
        if difx < 0 and dify < 0:
            angulo = np.pi + angulo
        elif difx < 0 and dify > 0:
            angulo = np.pi - angulo
        elif difx > 0 and dify < 0:
            angulo = 2*np.pi - angulo

        err = (angulo - self.yaw)

        if abs(err) > np.pi/6:
            self.twist.angular.z = 0.1

        else:
            anda = True
            self.twist.angular.z = float(err)*30/self.kp

        if abs(dify) <= 0.1 and abs(difx) <= 0.1:
            self.twist = Twist()

        elif anda:
            self.twist.linear.x = 0.1




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


