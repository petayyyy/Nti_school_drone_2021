# -*- coding: utf-8 -*-
# Обязательная конструкция для работы с кириллицей

# Инициализация библиотек
import numpy as np
import math
import cv2
import time
#import roslib
import sys
import rospy
from pyzbar import pyzbar
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from clover.srv import SetLEDEffect

# Инициализация сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

# 
class Main_Config():                                                                                              
    def __init__(self, simulator=False):
	# Создаем ноду
        rospy.init_node('detecting', anonymous=True)   
	# Создаем топик "Detect" для вывода измененного изображения(обводка распознаных объектов)
        self.image_pub = rospy.Publisher("Detect",Image,queue_size=10)                                    
        
	# Если мы летаем в симуляторе, то класс нужно вызывать со значением True (Пример: col = Main_Config(True) )
        self.simulator = simulator
	# Присваем значения цветов в соответствии со средой симулятор/реальный мир
        # Ваши параметры могут разниться, не забудьте подобрать новые
	if self.simulator:
	    # Параметры для симулятора
            self.red_low = np.array([0,0,240])                                                                             
            self.red_high = np.array([10,10,255])                                                                          
		
            self.blue_low = np.array([240,0,0])
            self.blue_high = np.array([255,10,10])

            self.yellow_low = np.array([0,240,240])
            self.yellow_high = np.array([10,255,255])
        else:
	    # Параметры для реального мира
            self.red_low = np.array([55,55,170])                            
            self.red_high = np.array([135,125,255])                                                                           

            self.blue_low = np.array([120,90,0])                                                                             
            self.blue_high = np.array([210,140,80])

            self.yellow_low = np.array([10,160,160])                                                                            
            self.yellow_high = np.array([120,230,220])
	
	# Задаем параметры для создания масок цвета
        self.st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
        self.st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5)) 
	# Создаем видео для отладки цветовых порогов и кода
        self.out = cv2.VideoWriter('Video_output.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240))
        
	# Переменные условий начала действий
	# Отвечает за рапознавание положения навигационной стрелки
        self.Arrow = False  
	# Отвечает за рапознавание Qr кода
        self.Qr = False
	# Отвечает за рапознавание цвета навигационной стрелки
        self.Color = False
	# Отвечает за рапознавание стеллажа
        self.Dron_point = False
        
	# Переменные хранящие данные
	# Номер заказа
        self.order = -1
	# Координаты препятствий
        self.col_ar = []
	# Координата навигационной стрелки
        self.nav = []
	# Координаты маршрута, построенного в обход препятствий
        self.mas = []
	# Высота стелллажа
        self.zz = 0.5
	# Координаты положения стеллажа относительно матрицы камеры
        self.cx, self.cy = -1, -1
	# Цвет навигационной стрелки
        self.color_arrow = 'black'
	# Все возможные значение навигационной стрелки, сектора
        self.sect = ['Sector B','Sector D','Sector A','Sector C']
        # Значение навигационной стрелки, сектор
	self.arrow = 'Sector D'
	# Переменная необходимая для работы с изображением из топика
        self.bridge = CvBridge()
	
	# Подписание на топик, содержащий изображение
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)
    def navigate_wait(self, x=0, y=0, z=0, yaw=math.radians(90), speed=1, frame_id='aruco_map', auto_arm=False, tolerance=0.15):
	# Функция полета до указанной точки
	# Летим до указаноой точки
	# И пока не прилетим, не выходим из цикла 
        navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
        while not rospy.is_shutdown():
	    # Берем координаты коптера в пространстве
            telem = get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.1)
    def navigate_mas(self):
	# Функция полета по координатам маршрута, построенного в обход препятствий
        for x, y in self.mas:
            self.navigate_wait(x=x*0.2, y=y*0.2, z=1.5, frame_id='aruco_map')
            if math.sqrt((self.nav[0]-x*0.2) ** 2 + (self.nav[1]-y*0.2) ** 2) < 0.2: break
    def normalize(self, x, y):
	# Функция нормализации координат
        return x / math.sqrt(x ** 2 + y ** 2), y / math.sqrt(x ** 2 + y ** 2)
    def land_on_dronpoint(self):
        # Функция посадки на стеллаж
        x0, y0 = 320 // 2, 240 // 2
	# Пока дрон не над центром стеллажа
        while math.sqrt((x0 - self.cx) ** 2 + (y0 - self.cy) ** 2) > 20:
	    # Берем координаты коптера в пространстве
            telem = get_telemetry(frame_id='aruco_map')
	    # Создаем вектор движения до стеллажа, с скоростью полета
            dx, dy = self.normalize(self.cx - x0, self.cy - y0)
            dx /= 15
            dy /= 15
            dy = -dy
            # Летим по созданному вектору к центру стеллажа
	    set_position(x=telem.x + dx, y=telem.y + dy, z=self.zz+0.2, yaw=math.radians(90), frame_id='aruco_map')
            rospy.sleep(0.1)
        # Когда прилетели производим индикацию, в соответствии с цветом навигационной стрелки
        if self.color_arrow == 'black':
            set_effect(effect='fade', r=255, g=255, b=255)
        elif self.color_arrow == 'red':
            set_effect(effect='fade', r=255, g=0, b=0)
        elif self.color_arrow == 'blue':
            set_effect(effect='fade', r=0, g=0, b=255)
        elif self.color_arrow == 'yellow':
            set_effect(effect='fade', r=255, g=255, b=0)
	# Садимся
        land()
        rospy.sleep(1)
	# Выключаем двигатели
        arming(False)
	# Производим корректную световую индикацию
	rospy.sleep(5)
	# Выключаем индикацию
	set_effect(effect='fade', r=0, g=0, b=0)
	# Выводим корректно сообщение о доставке
	print('{} delivered in {}'.format(self.order,self.arrow))
    def dop_oblet(self):
	# Функция находящая дополнительные препятствия, на случай когда расстояние между краями двух препятствий меньше размеров дрона 
        lenn = len(self.col_ar)
        for i in range(lenn-1):
            for j in range(i+1,lenn): 
                if i!=j and ((self.col_ar[i][0] - self.col_ar[j][0])**2 + (self.col_ar[i][1] - self.col_ar[j][1])**2)**0.5 <= 0.8:  
                    self.col_ar.append([min(self.col_ar[i][0],self.col_ar[j][0]) + abs(self.col_ar[i][0] - self.col_ar[j][0])/2, min(self.col_ar[i][1],self.col_ar[j][1]) + abs(self.col_ar[i][1] - self.col_ar[j][1])/2])    
    def check(self,x,y):
	# Функция проверки координаты на возможность подлета в нее, учитывая препятствия
        if True in [True for i in range(len(self.col_ar)) if ((x*0.2-self.col_ar[i][0])**2 + (y*0.2-self.col_ar[i][1])**2)**(1/2) < 0.4]: return True
        else: return False
    def check_line(self, a=1,b=1):
	# Функция находящая координаты конца вектора, со сдвигом по двум осям, учитывая препятствия 
        for i in range(17):
            if a == 0:
                if self.check(self.nav_x+i,self.f_y + abs(self.f_x-self.nav_x)) == False and self.nav_x+i <= 12:
                    self.v1 = self.nav_x+i
                    if b == 1: self.v2 = self.f_y + abs(self.f_x-self.nav_x)
                    else: self.v2 = self.f_y
                    return True
                elif self.check(self.nav_x-i,self.f_y + abs(self.f_x-self.nav_x)) == False and self.nav_x-i >= 0:
                    self.v1 = self.nav_x-i
                    if b == 1: self.v2 = self.f_y + abs(self.f_x-self.nav_x)
                    else: self.v2 = self.f_y
                    return True
            else:
                if self.check(self.f_x + abs(self.f_y-self.nav_y),self.nav_y+i) == False and self.nav_y+i <= 12:
                    self.v2 = self.nav_y+i
                    if b == 1 : self.v1 = self.f_x + abs(self.f_y-self.nav_y)
                    else: self.v1 = self.f_x
                    return True
                elif self.check(self.f_x + abs(self.f_y-self.nav_y),self.nav_y-i) == False and self.nav_y-i >= 0:
                    self.v2 = self.nav_y-i
                    if b == 1: self.v1 = self.f_x + abs(self.f_y-self.nav_y)
                    else: self.v1 = self.f_x
                    return True
            return False
    def check_circle(self):
	# Функция находящая координаты начала вектора, со сдвигом по двум осям, учитывая препятствия 
        for i in range(17):
            if self.det_line(self.f_x+i,self.f_y,self.v1,self.v2) == True and self.f_x+i <= 16 and self.det_line(self.f_x,self.f_y,self.f_x+i,self.f_y) == True:
                self.mas.append([self.f_x+i,self.f_y])
                self.mas.append([self.v1,self.v2])
                self.f_x, self.f_y = self.v1, self.v2
                return True
            elif self.det_line(self.f_x-i,self.f_y,self.v1,self.v2) == True and self.f_x-i >= 0 and self.det_line(self.f_x,self.f_y,self.f_x-i,self.f_y) == True:
                self.mas.append([self.f_x-i,self.f_y])
                self.mas.append([self.v1,self.v2])
                self.f_x, self.f_y = self.v1, self.v2
                return True
            elif self.det_line(self.f_x,self.f_y+i,self.v1,self.v2) == True and self.f_y+i <= 12 and self.det_line(self.f_x,self.f_y,self.f_x,self.f_y+i) == True:
                self.mas.append([self.f_x,self.f_y+i])
                self.mas.append([self.v1,self.v2])
                self.f_x, self.f_y = self.v1, self.v2
                return True
            elif self.det_line(self.f_x,self.f_y-i,self.v1,self.v2) == True and self.f_y-i >= 0 and self.det_line(self.f_x,self.f_y,self.f_x,self.f_y-i) == True:
                self.mas.append([self.f_x,self.f_y-i])
                self.mas.append([self.v1,self.v2])
                self.f_x, self.f_y = self.v1, self.v2
                return True
        return False
    def det_line(self,x1,y1,x2,y2,r=0.39):
	# Функция проверяющая наличие касания прямой окружности
        x1,y1,x2,y2 = x1*0.2,y1*0.2,x2*0.2,y2*0.2
        for x,y in self.col_ar:
            try:
                k = (y1 - y2)/(x1 - x2)
                b0 = y1 - k*x1
                a = k**2 + 1
                b = 2*k*(b0 - y) - 2*x
                c = (b0 - y)**2 + x**2 - r**2
                delta = b**2 - 4*a*c
                if delta >= 0: return False
            except: pass
        return True
    def vector_x(self):
	# Функция находящая координаты конца вектора, без сдвига по оси y, учитывая препятствия 
        self.v1,self.v2 = self.nav_x, self.nav_y
        if self.check_circle() == False:
            for i in range(12):
                self.v1,self.v2 = self.nav_x+i, self.nav_y
                if self.check_circle(): break
                self.v1,self.v2 = self.nav_x-i, self.nav_y
                if self.check_circle(): break
    def vector_y(self): 
	# Функция находящая координаты конца вектора, без сдвига по оси x, учитывая препятствия 
        self.v1,self.v2 = self.nav_x, self.nav_y
        if self.check_circle() == False:
            for i in range(12):
                self.v1,self.v2 = self.nav_x, self.nav_y+i
                if self.check_circle(): break
                self.v1,self.v2 = self.nav_x, self.nav_y-i
                if self.check_circle(): break

    def navigate_avoidece(self, start, finish):
	# Функция облета препятствий
	# Координаты конечной точки полета
        self.nav_x, self.nav_y = (finish[0]*100)//20, (finish[1]*100)//20
	# Координаты движения дрона
        self.f_x, self.f_y = start[0]//0.2, start[1]//0.2
	# Переменная, хранящая значения колличества координат для облета, чтобы коптер не зациклился
        h = 5
        while (self.f_x != self.nav_x or self.f_y != self.nav_y) and h > 0:
            h-=1
	    if self.f_x == self.nav_x:
	        # Если мы выравнялись по оси x
		# То выравниваемся по y
                self.vector_y()
            elif self.f_y == self.nav_y:
		# Если мы выравнялись по оси y
		# То выравниваемся по x
                self.vector_x()
            else:
		# Если мы не выравнялись не по одной из осей
		# То ищем самую кратчайшую ось для выравнивания
                if self.nav_x < self.nav_y:
		    # И пробуем выравнивается по x 
                    self.check_line(a=0)
                else: 
		    # И пробуем выравнивается по y
                    self.check_line(a=1)
                self.check_circle()
	
    def find_arrow(self,im):
	# Функция распознавания положения навигационной стрелки
	# Берем модели стрелок
	samples = np.loadtxt('generalsamples.data', np.float32)
        responses = np.loadtxt('generalresponses.data', np.float32)
        responses = responses.reshape((responses.size, 1))
        model = cv2.ml.KNearest_create()
        model.train(samples, cv2.ml.ROW_SAMPLE, responses)
        
	# Создаем копию изображения, для обводки навигационной стрелки
        out = im.copy()
	# Создаем маску 
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        kernal = np.ones((5, 5), np.uint8)
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
	# Ищем контуры стеллажа навигационной стрелки
	contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
        num = 0
        arr = []
        
	# Проходимся по контурам
        for cnt in contours:
	    # Берем координаты центра навигационной стрелки, относительно матрицы камеры
            [x, y, w, h] = cv2.boundingRect(cnt)
            try:
		# Если нашли, то обводим навигационною стрелку
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
		
                roi = thresh[y:y + h, x:x + w]
                roismall = cv2.resize(roi, (10, 10))
                roismall = roismall.reshape((1, 100))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
                num = int(results[0][0])
                arr.append((cnt, dists[0][0], num))
            except: pass
	
        # Записываем ее значение в переменную, для дальнейшей работы с ней
        need_arr = min(arr, key=lambda x: x[1])
        num = need_arr[2]
	# Обводим на изображении навигационную стрелку
        cv2.drawContours(out, [need_arr[0]], -1, (255,105,180), 3)
	# Выводим в топик изменненое изображение
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        except CvBridgeError as e:
            print(e)
	
	# Заносим в переменную значение положения навигационной стрелки
        self.arrow = self.sect[num]
	# Выводим в терминал сообщение о результатах распознавания
        print('{} required'.format(self.arrow))
	# Останавливаем поиск положения навигационной стрелки
	self.Arrow = False
    def color(self,mask,a):
	# Функция распознавания цвета навигационной стрелки
	# Создаем маску
        thresh = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self.st2)
	# Ищем контуры цветного объекта(навигационной стрелки)
        cnt = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]   
	# И проходим по ним
        for c in cnt:    
            try: 
		# Определяем колличество пикселей занятых цветом
                moments = cv2.moments(c, 1)       
                sum_pixel = moments['m00']
		# Если их больше 300, то мы нашли цвет навигационной стрелки
                if sum_pixel > 300:
		    # Заносим в переменную значение цвета навигационной стрелки
                    self.color_arrow = a
	            # Останавливаем поиск цвета навигационной стрелки
                    self.Color = False
            except:pass
    def dronpoint_detect(self,img):
	# Функция распознавания цвета навигационной стрелки
	# Создаем маску
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (50, 55, 50), (85, 255, 255))
	# Ищем контуры стеллажа
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
	# Если контуры есть
        if len(contours) != 0:
	    # Берем наибольший, тот который ближе всего к камере
            self.cnt = max(contours, key=cv2.contourArea)
            # Находим координаты центра стеллажа
            [x, y, w, h] = cv2.boundingRect(self.cnt) 
            self.cx = x + w // 2
            self.cy = y + h // 2
	    # Возращаем True если нашли стеллаж
            return True
    def sect_fly(self):
	# Функция полета от навигационной стрелки до стеллажа
	# Выбираем направление движения в соответствии с положение навигационной стрелки, сектором
        if self.arrow == 'Sector A':
            self.y_f = 0.4
            self.x_f = 0
        elif self.arrow == 'Sector B':
            self.y_f = -0.4
            self.x_f = 0
        elif self.arrow == 'Sector C':
            self.y_f = 0
            self.x_f = 0.4
        elif self.arrow == 'Sector D':
            self.y_f = 0
            self.x_f = -0.4
        self.mas = []
	# Берем координаты коптера в пространстве
        telem = get_telemetry(frame_id='aruco_map')
	# Если координата не приводит к столкновению с препятствиями, летим подальше, чтобы не принять стеллажи из других секторов за необходимый
	if self.check(telem.x+self.x_f*4, telem.y+self.y_f*4)== False:
	    # Летим еще подальше, чтобы не столкнуться с препятствиями
            self.navigate_avoidece([telem.x,telem.y],[telem.x+self.x_f*6, telem.y+self.y_f*6])
        else: self.navigate_avoidece([telem.x,telem.y],[telem.x+self.x_f*4, telem.y+self.y_f*4])
        self.navigate_mas()
        for i in range(9):
	    # Начинаем поиск стеллажа
	    self.Dron_point = True
            rospy.sleep(1)
	    # Если нашли стеллаж, выходим из функции
            if self.cx != -1 : break
            # Заканчиваем поиск стеллажа
            self.Dron_point = False
	    self.mas = []
	    # Берем координаты коптера в пространстве
            telem = get_telemetry(frame_id='aruco_map')
            # И если координата не приводит к столкновению с препятствиями, летим в нее
            if self.check(telem.x+self.x_f*i, telem.y+self.y_f*i)== False and telem.x+self.x_f*i <= 3.2 and telem.x+self.x_f*i >= 0 and telem.y+self.y_f*i <= 2.4 and telem.y+self.y_f*i >= 0:
                self.navigate_avoidece([telem.x,telem.y],[telem.x+self.x_f*i, telem.y+self.y_f*i])
    def callback(self,data):
	# Функция обрабатывающая значение из топика, в ней происходит обработка изображения
	# Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try - для отсутствия ошибки если топик будет пустой)
        try:                                                 
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:pass
	# Если в реальном мире, выравниваем изображение
        if self.simulator == False:
            img = cv2.undistort( img,np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]), np.array([ 2.15356885e-01,  -1.17472846e-01,  -3.06197672e-04,-1.09444025e-04,  -4.53657258e-03,   5.73090623e-01,-1.27574577e-01,  -2.86125589e-02,   0.00000000e+00,0.00000000e+00,   0.00000000e+00,   0.00000000e+00,0.00000000e+00,   0.00000000e+00]),np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]))
        # Записываем видео, для отладки цветовых порогов и кода
	self.out.write(img)
	# Начинаем распознавать Qr код
        if self.Qr == True:
	    # Производим анилиз изображения на наличие Qr кода
            barcodes  = pyzbar.decode(img) 
	    # Если нашли Qr код, то работем с ним
            if barcodes:
                for bar in barcodes:
		    # Если до этого Qr код не распознавали
                    if self.order == -1:
			# Получаем данные из Qr кода
                        self.qr_data = (bar.data.decode("utf-8")).split('\n')
                        for i in range (0,len(self.qr_data[0].split())-1,2):
			    # Корректно выводим координаты препятствий
                            print('Column area x={}, y={}'.format(self.qr_data[0].split()[i],self.qr_data[0].split()[i+1]))
			    # И записываем в переменную, для дальнейшей работы с ними
                            self.col_ar.append([float(self.qr_data[0].split()[i]), float(self.qr_data[0].split()[i+1])])
			# Корректно выводим координату навигационной стрелки
                        print('Navigation area x={}, y={}'.format(self.qr_data[1].split()[0],self.qr_data[1].split()[-1]))
			# И записываем в переменную, для дальнейшей работы с ней
                        self.nav.append(float(self.qr_data[1].split()[0]))
                        self.nav.append(float(self.qr_data[1].split()[-1]))
			# Корректно выводим номер заказа
                        print('Order number: {}'.format(self.qr_data[-1]))
			# И записываем в переменную, для дальнейшей работы с ним
                        self.order = int(self.qr_data[-1])
			# Запускаем функцию, находящую дополнительные препятствия
                        self.dop_oblet()
		    # Ищем координаты Qr кода 
                    (x, y, w, h) = bar.rect
		    # Обводим на изображении Qr код
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255,105,180), 2)
        # Начинаем распознавать цвет навигационной стрелки
	if self.Color:
	    # Ищем цвета на изображении
	    # Красный
            self.color(cv2.inRange(img, self.red_low, self.red_high),'red')          
	    # Желтый
            self.color(cv2.inRange(img, self.yellow_low, self.yellow_high),'yellow')
	    # Синий
            self.color(cv2.inRange(img, self.blue_low, self.blue_high),'blue')
	    
            # В соответствии с цветом, выставляем высоту стеллажа
	    if self.color_arrow == 'yellow': self.zz = 0.25
	    elif self.color_arrow == 'blue': self.zz = 0.75
	    elif self.color_arrow == 'red': self.zz = 1
        # Начинаем распознавать положение навигационной стрелки
	if self.Arrow:
            self.find_arrow(img.copy())
	# Начинаем поиск на изображении стеллажа
        if self.Dron_point:
            if self.dronpoint_detect(img.copy()):
		# Если нашли стеллаж, обводим его
                cv2.drawContours(img, [self.cnt], 0, (255,105,180), 2)

        # Выводим в топик изменненое изображение
	try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)
