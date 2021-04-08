# -*- coding: utf-8 -*-
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

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

class Main_Config():                                                                                              
    def __init__(self, simulator=False):
        rospy.init_node('detecting', anonymous=True)                                                              
        self.image_pub = rospy.Publisher("Detect",Image,queue_size=10) # ???????? ?????? ??? ?????????? ????????????? ???????????                                    
        
        self.simulator = simulator
        if self.simulator: # ????????? ??? ??????????
            self.red_low = np.array([0,0,240])                                                                             
            self.red_high = np.array([10,10,255])                                                                          
		
            self.blue_low = np.array([240,0,0])
            self.blue_high = np.array([255,10,10])

            self.yellow_low = np.array([0,240,240])
            self.yellow_high = np.array([10,255,255])

            #self.green_low = np.array([50, 55, 50]) 
            #self.green_high = np.array([85, 255, 255])
            self.green_low = np.array([16, 88, 46])
            self.green_high = np.array([90, 255, 255])
        else: # ????????? ??? ????????? ??????
            self.red_low = np.array([55,55,170])                            
            self.red_high = np.array([135,125,255])                                                                           

            self.blue_low = np.array([120,90,0])                                                                             
            self.blue_high = np.array([210,140,80])

            self.yellow_low = np.array([10,160,160])                                                                            
            self.yellow_high = np.array([120,230,220])
        self.st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
        self.st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))   
        self.out = cv2.VideoWriter('Video_output.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240)) # ?????? ????? ??? ??????? ???? ? ???????? ???????
        
        self.Arrow = False   
        self.Qr = False
        self.Color = False
        self.Dron_point = False
        
        self.order = -1
        self.col_ar = []
        self.nav = []
        self.mas = []
        self.zz = 1
        self.cx, self.cy = -1, -1
        self.color_arrow = 'black'
        self.sect = ['Sector B','Sector D','Sector A','Sector C']
        self.arrow = 'Sector D'
        self.bridge = CvBridge()                                                                                     
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)  # ?????????? ?? ????? ???????????
    def navigate_wait(self, x=0, y=0, z=0, yaw=math.radians(90), speed=1, frame_id='aruco_map', auto_arm=False, tolerance=0.15):
        navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.1)
    def navigate_mas(self):
        for x, y in self.mas:
            self.navigate_wait(x=x*0.2,y=y*0.2,z=self.zz, frame_id='aruco_map')
    def normalize(self, x, y):
        return x / math.sqrt(x ** 2 + y ** 2), y / math.sqrt(x ** 2 + y ** 2)
    def land_on_dronpoint(self):
        x0, y0 = 320 // 2, 240 // 2
        while math.sqrt((x0 - self.cx) ** 2 + (y0 - self.cy) ** 2) > 20:
            telem = get_telemetry(frame_id='aruco_map')
            print(math.sqrt((x0 - self.cx) ** 2 + (y0 - self.cy) ** 2))
            dx, dy = self.normalize(self.cx - x0, self.cy - y0)  # get final motion vector 
            dx /= 15  # limit the speed
            dy /= 15
            dy = -dy  # the y-axis of the frame is directed in the opposite direction of the y-axis of the marker map
            set_position(x=telem.x + dx, y=telem.y + dy, z=self.zz, yaw=math.radians(90), frame_id='aruco_map')
            rospy.sleep(0.1)
        if self.color_arrow == 'black':
            set_effect(effect='fade', r=255, g=255, b=255)
        elif self.color_arrow == 'red':
            set_effect(effect='fade', r=255, g=0, b=0)
        elif self.color_arrow == 'blue':
            set_effect(effect='fade', r=0, g=0, b=255)
        elif self.color_arrow == 'yellow':
            set_effect(effect='fade', r=255, g=255, b=0)
        land()
        rospy.sleep(1)
        arming(False)
	rospy.sleep(5)
	set_effect(effect='fade', r=0, g=0, b=0)
	print('{} delivered in {}'.format(self.order,self.arrow))

    def dop_oblet(self):
        lenn = len(self.col_ar)
        for i in range(lenn-1):
            for j in range(i+1,lenn): 
                if i!=j and ((self.col_ar[i][0] - self.col_ar[j][0])**2 + (self.col_ar[i][1] - self.col_ar[j][1])**2)**0.5 <= 0.8:  
                    self.col_ar.append([min(self.col_ar[i][0],self.col_ar[j][0]) + abs(self.col_ar[i][0] - self.col_ar[j][0])/2, min(self.col_ar[i][1],self.col_ar[j][1]) + abs(self.col_ar[i][1] - self.col_ar[j][1])/2])    
    def check(self,x,y):
        if True in [True for i in range(len(self.col_ar)) if ((x*0.2-self.col_ar[i][0])**2 + (y*0.2-self.col_ar[i][1])**2)**(1/2) < 0.4]: return True
        else: return False
    def check_line(self, a=1,b = 1):
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
        self.v1,self.v2 = self.nav_x, self.nav_y
        if self.check_circle() == False:
            for i in range(12):
                self.v1,self.v2 = self.nav_x+i, self.nav_y
                if self.check_circle(): break
                self.v1,self.v2 = self.nav_x-i, self.nav_y
                if self.check_circle(): break
    def vector_y(self): 
        self.v1,self.v2 = self.nav_x, self.nav_y
        if self.check_circle() == False:
            for i in range(12):
                self.v1,self.v2 = self.nav_x, self.nav_y+i
                if self.check_circle(): break
                self.v1,self.v2 = self.nav_x, self.nav_y-i
                if self.check_circle(): break

    def navigate_avoidece(self, start, finish):
        self.nav_x, self.nav_y = (finish[0]*100)//20, (finish[1]*100)//20
        self.f_x, self.f_y = start[0]//0.2, start[1]//0.2
        h = 0
        while (self.f_x != self.nav_x or self.f_y != self.nav_y) and h < 5:
            h+=1
            if self.f_x == self.nav_x:
                self.vector_y()
            elif self.f_y == self.nav_y:
                self.vector_x()
            else:
                if self.nav_x < self.nav_y:
                    self.check_line(a=0)
                else: 
                    self.check_line(a=1)
                self.check_circle()
    
    def find_arrow(self,im):
        samples = np.loadtxt('generalsamples.data', np.float32)
        responses = np.loadtxt('generalresponses.data', np.float32)
        responses = responses.reshape((responses.size, 1))

        model = cv2.ml.KNearest_create()
        model.train(samples, cv2.ml.ROW_SAMPLE, responses)

        out = im.copy()

        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        kernal = np.ones((5, 5), np.uint8)

        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)

        # ?? ???? ???? ??? ??? ?? ????? ?????? ????? ???????, ?? ?????? ???????? ? ?????????? ????????????
        contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]

        cnt = max(contours, key=cv2.contourArea)

        num = 0
        arr = []

        for cnt in contours:
            [x, y, w, h] = cv2.boundingRect(cnt)
            try:
                #print("is contour")
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
                roi = thresh[y:y + h, x:x + w]
                roismall = cv2.resize(roi, (10, 10))
                roismall = roismall.reshape((1, 100))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
                num = int(results[0][0]) # ?
                arr.append((cnt, dists[0][0], num))
            except: pass

        need_arr = min(arr, key=lambda x: x[1])
        num = need_arr[2]
        cv2.drawContours(out, [need_arr[0]], -1, (255,105,180), 3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8")) # ????? ?????? ??? ?????? 1
        except CvBridgeError as e:
            print(e)
        self.arrow = self.sect[num]
        print('{} required'.format(self.arrow))
	self.Arrow = False
    def color(self,mask,a):
        thresh = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self.st2)
        cnt = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]   
        for c in cnt:    
            try: # ????????????? 
                moments = cv2.moments(c, 1)       
                sum_pixel = moments['m00']
                if sum_pixel > 300:
                    self.color_arrow = a
                    self.Color = False
                    print(a)
            except:pass
    def dronpoint_detect(self,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (50, 55, 50), (85, 255, 255))

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        if len(contours) != 0:
            self.cnt = max(contours, key=cv2.contourArea)
            [x, y, w, h] = cv2.boundingRect(self.cnt)  # getting the the coordinates of the bigest contour 
            self.cx = x + w // 2
            self.cy = y + h // 2
            return True
    def sect_fly(self):
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
        telem = get_telemetry(frame_id='aruco_map')
        self.navigate_wait(x=telem.x+self.x_f*4, y=telem.y+self.y_f*2, z =self.zz, frame_id='aruco_map')
        for i in range(9):
            telem = get_telemetry(frame_id='aruco_map')
            if self.check(telem.x+self.x_f*i, telem.y+self.y_f*i)== False and telem.x+self.x_f*i <= 3.2 and telem.x+self.x_f*i >= 0 and telem.y+self.y_f*i <= 2.4 and telem.y+self.y_f*i >= 0:
                self.navigate_avoidece([telem.x,telem.y],[telem.x+self.x_f*i, telem.y+self.y_f*i])
            #self.navigate_wait(x=telem.x+self.x_f*i, y=telem.y+self.y_f*i, z =self.zz, frame_id='aruco_map')
            self.Dron_point = True
            rospy.sleep(1)
            if self.cx != -1 : break
            self.Dron_point = False
    def callback(self,data):  
        try:                                                 
            img = self.bridge.imgmsg_to_cv2(data, "bgr8") # ?????? ???????????
        except:pass
        if self.simulator == False: # ???????????? ???????????
            img = cv2.undistort( img,np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]), np.array([ 2.15356885e-01,  -1.17472846e-01,  -3.06197672e-04,-1.09444025e-04,  -4.53657258e-03,   5.73090623e-01,-1.27574577e-01,  -2.86125589e-02,   0.00000000e+00,0.00000000e+00,   0.00000000e+00,   0.00000000e+00,0.00000000e+00,   0.00000000e+00]),np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]))
        self.out.write(img)
        if self.Qr == True:
            barcodes  = pyzbar.decode(img)    # ????????????? QR-?????
            if barcodes:    # ???? ??? ?? ???????? ????
                for bar in barcodes:       # ???????? ?? ???? QR ?????, ??????? ?? ?????
                    if self.order == -1:
                        self.qr_data = (bar.data.decode("utf-8")).split('\n') # ?????????? ? ?????????? ??????????, ??????????? ? ?????? ????
                        for i in range (0,len(self.qr_data[0].split())-1,2): 
                            print('Column area x={}, y={}'.format(self.qr_data[0].split()[i],self.qr_data[0].split()[i+1]))
                            self.col_ar.append([float(self.qr_data[0].split()[i]), float(self.qr_data[0].split()[i+1])])
                        print('Navigation area x={}, y={}'.format(self.qr_data[1].split()[0],self.qr_data[1].split()[-1]))
                        self.nav.append(float(self.qr_data[1].split()[0]))
                        self.nav.append(float(self.qr_data[1].split()[-1]))
                        print('Order number: {}'.format(self.qr_data[-1]))
                        self.order = int(self.qr_data[-1])
                        self.dop_oblet()
                        self.navigate_avoidece([0.4,0.8], self.nav)

                    (x, y, w, h) = bar.rect
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255,105,180), 2)
        if self.Color:
            self.color(cv2.inRange(img, self.red_low, self.red_high),'red')          #Red
            self.color(cv2.inRange(img, self.yellow_low, self.yellow_high),'yellow')   #Yellow
            self.color(cv2.inRange(img, self.blue_low, self.blue_high),'blue')          #Blue
        if self.Arrow:
            self.find_arrow(img.copy())
        if self.Dron_point:
            if self.dronpoint_detect(img.copy()):
                cv2.drawContours(img, [self.cnt], 0, (255,105,180), 2)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)
