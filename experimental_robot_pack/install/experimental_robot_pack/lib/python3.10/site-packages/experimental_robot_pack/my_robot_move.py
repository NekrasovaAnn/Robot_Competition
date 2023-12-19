from ast import Str
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
import time
from rclpy.action import ActionClient
import pkg_resources
import os


class Camera(Node):
    def __init__(self):
        super().__init__('depth_stopping_Node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Image, '/color/image', self.pose_callback, 1)
        self.p_ = self.create_publisher(String, '/robot_start', 1)
        self.pose_m = self.create_subscription(String, '/robot_start',self.mesg,1)
        self.timer = self.create_timer(0.2, self.move_robot)

        self.path = os.path.join(os.path.dirname(pkg_resources.resource_filename('experimental_robot_pack', "1")),"..","..",'..','..','share','experimental_robot_pack', 'sign')
        print(self.path)
        self.sign_l = cv2.imread(self.path + '/left.jpg')
        self.sign_r = cv2.imread(self.path + '/right.jpg')

        Camera.mstate = 0
        self.t = 0

        self.msg = np.zeros(1)
        self.E = 0
        self.old_e = 0
        
        self.Kp = 0.2#3.0
        self.Ki = 0.001#0.1
        self.Kd = 0.0#0.25
        

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
    def find_borders(self, img, line_num=420): 
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
        cv2.circle(img, (img_center, 420), 5, (0, 255, 0), -1)
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
    # size area - ожидаемые размеры знака, threshold - пороговый уровень уверенности
    def detect_sign(self, sign, area = [0, 460, 200, 700], size_area = [100, 100], threshold = 0.6): 
        image = self.msg
        [x1, x2, y1, y2] = area
        img = image[x1:x2, y1:y2]

        # cv2.imshow("k", img)
        # cv2.waitKey(10)

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sign = cv2.resize(sign, (size_area[0], size_area[1]), interpolation = cv2.INTER_AREA)  
        template = cv2.cvtColor(sign, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(gray_img,template,cv2.TM_CCOEFF_NORMED)  
        loc = np.where(res >= threshold)   
        for pt in zip(*loc[::-1]):  
            cv2.rectangle(img, pt,(pt[0] + size_area[0], pt[1] + size_area[1]),(0,255,255), 2)  
            return True
        return False # Возвращает true, если знак есть, false - если его нет


    # Проезжает перекресток
    def crossroad(self, direction):
        if direction == 1: # right
            img = self.msg
            img = img[0:len(img[0])-1, len(img[1])//2:len(img[1])-1]
            return self.find_centre(img)
        else: # left
            img = self.msg
            img = img[0:len(img[0])-1, 0:len(img[1])//2]
            return self.find_centre(img)


    # Решает, когда пора двигать робота
    def move_robot(self):
        message = Twist()
        k = 0

        if self.t == 0: 
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
        elif self.t == 1 : 
            detected_r = self.detect_sign(self.sign_r)
            detected_l = self.detect_sign(self.sign_l)
            if detected_r == False and detected_l == False : k = 1
            elif detected_r : # Поворот направо
                print("right")  
                self.t = 2
                k=0
            elif detected_l : # Поворот налево
                print("left")  
                self.t = 3
                k=0
        elif self.t == 2:
            error = self.crossroad(1)
            message.linear.x = 0.1
            out = self.PID(error)
            if error < 0 :
                message.angular.z = float(out)
            else : message.angular.z = float(-out)
            k=2
        elif self.t == 3:
            error = self.crossroad(2)
            message.linear.x = 0.1
            out = self.PID(error)
            if error < 0 :
                message.angular.z = float(out)
            else : message.angular.z = float(-out)
            k=2


        if k == 1:
            error = self.find_centre(self.msg)
            if error<5 and error>-5: # Ограничение на случайные колебания
                message.linear.x = 0.1
                message.angular.z = 0.0
            else:
                message.linear.x = 0.1
                out = self.PID(error)
                # print(error)
                # print(out)
                if error < 0 :
                    message.angular.z = float(out)
                else : message.angular.z = float(-out)
        elif k == 1: 
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



