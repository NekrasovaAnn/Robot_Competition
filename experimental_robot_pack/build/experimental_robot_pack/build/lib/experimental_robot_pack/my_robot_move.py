from ast import Str
from json import detect_encoding
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
import time
import pkg_resources
import os


class Camera(Node):
    def __init__(self):
        super().__init__('depth_stopping_Node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Image, '/color/image', self.pose_callback, 1)
        self.p_ = self.create_publisher(String, '/robot_start', 1)
        self.finish = self.create_publisher(String, '/robot_finish', 1)
        self.pose_m = self.create_subscription(String, '/robot_start',self.mesg,1)
        self.timer = self.create_timer(0.2, self.move_robot)

        self.path = os.path.join(os.path.dirname(pkg_resources.resource_filename('experimental_robot_pack', "1")),"..","..",'..','..','share','experimental_robot_pack', 'sign')
        print(self.path)
        self.sign_l = cv2.imread(self.path + '/left.jpg')
        self.sign_r = cv2.imread(self.path + '/right.jpg')

        Camera.mstate = 0
        self.t = 0
        self.timer = 0
        self.right = 1  # 1 - right, 0 - left, -1 - nothing

        self.msg = np.zeros(1)
        self.msg_depth = np.zeros(1)
        self.E = 0
        self.old_e = 0
        
        self.Kp = 0.25#3.0
        self.Ki = 0.001#0.1
        self.Kd = 0.001#0.25
        

    # Не двигаемся, пока нет разрешения
        # ros2 topic pub /robot_start std_msgs/msg/String "data: 'start'"
    def mesg(self, data):
        if data != 0:
            Camera.mstate = 1


    # Обработка изображения, поступающего с камеры
    def pose_callback(self, data): 
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data,desired_encoding=data.encoding)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.msg = img

    # def fixAngle(self, angle):
    #     return np.arctan2(np.sin(angle), np.cos(angle))

    def PID(self, target):

        d_x = target
        d_y = 0

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)

        # Error between the goal angle and robot angle
        alpha = g_theta - 0
        e = np.arctan2(np.sin(alpha), np.cos(alpha))

        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e

        angular_speed = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

        angular_speed = np.arctan2(np.sin(angular_speed), np.cos(angular_speed))

        self.E = self.E + e
        self.old_e = e

        return angular_speed


    # Находит границы дороги на конкретной(line_num) горизонтальной линии изображения img
    def find_borders(self, img, line_num=430): 
        lower_white = np.array([230, 230, 230])
        upper_white = np.array([255, 255, 255])
        white_mask = cv2.inRange(img, lower_white, upper_white)
        white_mask_up = white_mask[line_num]

        lower_yellow = np.array([0, 200, 200])
        upper_yellow = np.array([100, 255, 255])
        yellow_mask = cv2.inRange(img, lower_yellow, upper_yellow)
        yellow_mask_up = yellow_mask[line_num]

        white_indices = np.where(white_mask_up == 255)
        yellow_indices = np.where(yellow_mask_up == 255)

        if len(white_indices[0]) > 0:
            end_white = white_indices[0][0]
        else:
            end_white = -1

        if len(yellow_indices[0]) > 0:
            start_yellow = yellow_indices[0][-1]
        else:
            start_yellow = -1
            
        return start_yellow, end_white # Возвращает индексы левой и правой границы 


    # Находит относительное расстояние от местоположения до центра дороги
    def find_centre(self, image): 
        img = image
        #img - изображение с камеры(вид сверху), line_num - номер линии на которой искать центр
        width = len(img[0])

        left, right = self.find_borders(img)
        if (left == -1):
            left = 0
        if (right == -1):   
            right = width

        # road_width = right - left + 1
        road_center = (left + right + 1) // 2
        img_center = (width + 1) // 2
        distance = -road_center + img_center - 28  # 28 - разница между центром изображения и центром робота
        cv2.circle(img, (road_center, 420), 5, (255, 0, 0), -1)
        cv2.circle(img, (img_center, 450), 5, (0, 255, 0), -1)
        cv2.imshow("k", img)
        cv2.waitKey(10)
        return -distance


    # Обнаруживает зеленый сигнал на изображении img
    def detect_green(self):
        image = self.msg
        image = cv2.GaussianBlur(image, (3, 3), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([55, 200, 40])
        upper_green = np.array([60, 255, 120])

        mask = cv2.inRange(hsv, lower_green, upper_green)
        result = cv2.bitwise_and(image, image, mask=mask)
        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        if cv2.countNonZero(result) > 10:
            return True
        else:
            return False # Возвращает true если на изображении есть зеленый сигнал, false - если нет


    # Обнаруживает дорожный знак на изображении
    # area - зона поиска(левая, правая, верхняя, нижняя границы), sign - изображение знака, image - изображение на котором искать, 
    # distances - порог соотношения расстояний Хэмминга, threshold - пороговый уровень уверенности по количеству расстояний Хэмминга
    def detect_sign(self, sign, area=[50, 260, 250, 520], size_area=[95, 95], threshold=0.7):
        image = self.msg
        [x1, x2, y1, y2] = area
        img = image[x1:x2, y1:y2]
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sign = cv2.resize(sign, (size_area[0], size_area[1]), interpolation = cv2.INTER_AREA)  
        template = cv2.cvtColor(sign, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(gray_img,template,cv2.TM_CCOEFF_NORMED)  
        loc = np.where(res >= threshold)   
        for pt in zip(*loc[::-1]):  
            cv2.rectangle(img, pt,(pt[0] + size_area[0], pt[1] + size_area[1]),(0,255,255), 2)  
            return np.argmax(res)
        return 0 # Возвращает true, если знак есть, false - если его нет



    # Решает, когда пора двигать робота
    def move_robot(self):
        message = Twist()
        k = 0

        if self.t == 0: # определяет зеленый свет
            c = self.detect_green()
            if c : 
                s = String()
                s.data = "start"
                self.p_.publish(s)
                self.t = 1

                if Camera.mstate != 0:
                    k = 1
                else: k = 0
            else : self.t = 0
        elif self.t == 1 : # ищет знак поворота
            detected_r = self.detect_sign(self.sign_r)
            detected_l = self.detect_sign(self.sign_l, size_area=[112, 112])
            if detected_l < 0.7 and detected_r < 0.7 : k = 1
            elif detected_r > detected_l: # Поворот направо
                print("right")  
                self.t = 2
                self.right = 1
                print("право  ", detected_r)
                print("лево   ", detected_l)
                k=0
            elif detected_l > detected_r: # Поворот налево
                print("left")  
                self.t = 3
                self.right = 0
                print("право  ", detected_r)
                print("лево   ", detected_l)
                k=0
        elif self.t == 2:  # right
            if self.timer>35: self.t = 4
            elif self.timer > 20:
                message.linear.x = 0.005
                message.angular.z = -0.5
            else:
                message.linear.x = 0.1
                message.angular.z = 0.0
            k=2
            self.timer += 1
        elif self.t == 3:  # left
            if self.timer > 37: self.t = 4
            elif self.timer > 20:
                message.linear.x = 0.005
                message.angular.z = 0.5
            else:
                message.linear.x = 0.1
                message.angular.z = 0.0
            k=2
            self.timer += 1
        elif self.t == 4:  # едет по перекрестку
            k=1
            self.t = 5
            self.timer = 0
        elif self.t == 5: 
            if self.right == 1:
                if self.timer < 82 : 
                    k=1
                else: 
                    self.timer = 0
                    self.t = 6
            elif self.right == 0:
                if self.timer < 68 : 
                    k=1
                else: 
                    self.timer = 0
                    self.t = 6
            self.timer += 1
            # k=0
        elif self.t == 6 :  # выезжает с пеекрестка
            k = 2
            if self.right == 1: 
                if self.timer < 20:
                    message.linear.x = 0.065
                    message.angular.z = -0.5
                elif self.timer < 33:
                    message.linear.x = 0.1
                    message.angular.z = 0.0
                else : 
                    self.timer = 0
                    self.t = 9
            else :
                if self.timer < 20:
                    message.linear.x = 0.065
                    message.angular.z = 0.5 
                elif self.timer < 35:
                    message.linear.x = 0.1
                    message.angular.z = 0.0
                else : 
                    self.timer = 0
                    self.t = 9
            self.timer += 1
        # elif self.t == 7:
        #     if self.timer < 155 : 
        #         self.right = 0
        #         k=1
        #     else :  
        #         self.timer = 0
        #         self.t = 8
        #     self.timer += 1   
        # elif self.t == 8:
        #     self.t = 9 
        elif self.t == 9 : # финиш
            s = String()
            s.data = "Name finished"
            self.p_.publish(s)
            k = 0

        if k == 1:
            error = self.find_centre(self.msg)
            if error<5 and error>-5: # Ограничение на случайные колебания
                message.linear.x = 0.1
                message.angular.z = 0.0
            else:
                message.linear.x = 0.1
                if self.right == 1: out = self.PID(error)
                else : out = self.PID(-error)
                if error < 0 : message.angular.z = float(out)
                else : message.angular.z = float(-out)
        elif k == 0: 
            message.linear.x = 0.0
            message.angular.z = 0.0

        self.publisher_.publish(message)

                
                


# Starting point of the code
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Camera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    


if __name__ == "__main__":
    main()



