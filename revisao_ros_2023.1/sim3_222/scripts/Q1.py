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
            "green": {
                "lower": np.array([75, 255, 255],dtype=np.uint8),
                "upper": np.array([79, 255, 255],dtype=np.uint8)
            },	
        }


        self.kernel = np.ones((5,5),np.uint8)
        self.n1 = random.randint(1,3) 
        
        # Subscribers
        self.bridge = CvBridge()
        self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size=1, buff_size = 2**24)
        
        #para a garra
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)	
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

        self.cmd_vel_pub.publish(Twist())

        # É A QUE VOU DIZER, QUAL ATIVIDADE O ROBO ESTÁ FAZENDO NO MOMENTO
        self.state = 1
        # o que o robo ta vendo 
        self.selected_mod = None

        self.robot_state = "rotate"
        self.robot_machine = {
            "rotate": self.rotate,
            "checar": self.checar,
            "center_on_coord": self.center_on_coord,
            "para": self.para,
            "garra" : self.controla_garra,           
        }

        self.mobile_machine = {
             "aproxima" : self.aproxima,
        }

        self.color_machine = {
             "aproxima" : self.aproxima,
        }
        self.kp = 100



	
    def laser_callback(self, msg: LaserScan) -> None:
        self.laser_msg = np.array(msg.ranges).round(decimals=2)
        self.laser_msg[self.laser_msg == 0] = np.inf


        self.laser_forward = np.min(list(self.laser_msg[0:5]) + list(self.laser_msg[354:359]))
        self.laser_backwards = np.min(list(self.laser_msg[175:185]))

    def color_segmentation(self, hsv: np.ndarray, lower_hsv: np.ndarray, upper_hsv: np.ndarray,) -> Point:
        """ 
        Use HSV color space to segment the image and find the center of the object.

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
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            img = cv_image.copy()
        except CvBridgeError as e:
            print(e)
        _,self.resultados = detect(net, img, CONFIDENCE, COLORS, CLASSES)

        self.w = img.shape[1] # [h,w,p] --- > [1] = w

        self.yellow, area_y = self.color_segmentation(hsv, self.color_param["yellow"]["lower"], self.color_param["yellow"]["upper"])
        self.magenta, area_m = self.color_segmentation(hsv, self.color_param["magenta"]["lower"], self.color_param["magenta"]["upper"])
        self.green, area_g = self.color_segmentation(hsv, self.color_param["green"]["lower"], self.color_param["green"]["upper"])

        self.mobile = {1:"car",2:"bicycle",3:"horse"}
        self.cores = {1:"yellow",2:"magenta",3:"green"}
        self.centro_cor = {1:self.yellow,2:self.magenta,3:self.green}
        # atividade 1: verificar mobile net 
        if self.state == 1:
            #se a lista de mobilenet for diferente de zero
            if len(self.resultados) != 0:
                #verifica se é a caixa sorteada 
                if self.resultados[0][0] == self.mobile[self.n1]:
                    #fala pro robo que é a caixa sorteada
                    self.selected_mod = self.mobile[self.n1]
                    #da o comando pra ele ir checar 
                    self.robot_state = "checar"

        #atividade 2: seguir as caixas de cor 
        elif self.state == 2:
            #pego no meu dicionario de centro de cor a cor sorteada e verifico se a cordenada x do centro é diferente de zero
            if self.centro_cor[self.n1][0] != 0:
                #falo pro robo qual é a cor da caixa
                self.selected_mod = self.cores[self.n1]
                self.robot_state = "checar"




    def rotate(self) -> None:
        """
        Rotate the robot
        """
        self.twist.angular.z = 0.5
        ...

    def checar(self) -> None:
        """
        Stop the robot
        """
        if self.selected_mod == "car": 
            self.robot_machine.update(self.mobile_machine)
            self.robot_state = "aproxima"
        elif self.selected_mod == "bicycle":
            self.robot_machine.update(self.mobile_machine)
            self.robot_state = "aproxima"
        elif self.selected_mod == "horse":
            self.robot_machine.update(self.mobile_machine)
            self.robot_state = "aproxima"
        elif self.selected_mod == "yellow":
            self.robot_machine.update(self.color_machine)
            self.robot_state = "aproxima"
        elif self.selected_mod == "magenta":
            self.robot_machine.update(self.color_machine)
            self.robot_state = "aproxima"
        elif self.selected_mod == "green":
            self.robot_machine.update(self.color_machine)
            self.robot_state = "aproxima"
        



    def aproxima(self) -> None:
        self.center_on_coord()  #centraliza imagem com o centro da caixa      
        if self.laser_forward >= 0.5: #enquanto for maior que 0.5 vai pra frente e da velocidde linear de 0.2
            self.twist.linear.x = 0.2

        else: #quando for menor ou igual a 0.5 vai pra função parar
            self.robot_state = "para"


    def para(self) -> None:
        self.twist = Twist() 
        if self.state == 1: 
            self.state = 2
            self.robot_state = "rotate"
        elif self.state == 2:
            self.state = 3
            self.robot_state = "garra"
        elif self.state == 3: 
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
            


    def center_on_coord(self):
        self.twist = Twist()
        err = 0
        
        if self.selected_mod == "car":
            err = self.w/2 - ((self.resultados[0][2][0] + self.resultados[0][3][0])/2)
        elif self.selected_mod == "bicycle":
            err = self.w/2 - ((self.resultados[0][2][0] + self.resultados[0][3][0])/2)
        elif self.selected_mod == "horse":
            err = self.w/2 - ((self.resultados[0][2][0] + self.resultados[0][3][0])/2)
        elif self.selected_mod == "yellow":
            err = self.w/2 - self.centro_cor[self.n1][0]
        elif self.selected_mod == "magenta":
            err = self.w/2 - self.centro_cor[self.n1][0]
        elif self.selected_mod == "green":
            err = self.w/2 - self.centro_cor[self.n1][0]
        
        self.twist.angular.z = float(err) / self.kp # quanto maior o kp mais preciso vai ser o seu robo, quanto menor o kp menos preciso 

    def controla_garra(self):

        #levanta
        self.ombro.publish(1.0) # numeros postivos 
        rospy.sleep(1.0)
        #abaixa
        self.ombro.publish(-1.5) #numeros negativos
        rospy.sleep(1.0)

        if self.state == 3:
            self.robote_state = "para"

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
