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

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land = rospy.ServiceProxy('land', Trigger)

class ArrowDetecting():                                                                                              
    def __init__(self, simulator):
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
        
        self.order = -1
        self.col_ar = []
        self.nav = []
        self.color_arrow = 'black'
        self.sect = ['Sector B','Sector D','Sector A','Sector C']
        self.arrow = 'Sector D'
        self.bridge = CvBridge()                                                                                     
        self.image_sub = rospy.Subscriber("main_camera/image_raw",Image,self.callback)  # ?????????? ?? ????? ???????????
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
        #cv2.drawContours(out, [need_arr[0]], -1, (255, 105, 180), 3)
        self.arrow = self.sect[num]
        print('{} required'.format(self.arrow))

    def color(self,mask,a):
        thresh = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self.st2)
        cnt = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]   
        #print(cnt)
        for c in cnt:    
            try: # ????????????? 
                moments = cv2.moments(c, 1)       
                sum_pixel = moments['m00']
                if sum_pixel > 300:
                    self.color_arrow = a
                    self.Color = False
                print(a)
                    #cv2.drawContours(img, [c], 0, (193,91,154), 2)
            except:pass
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
                            self.col_ar.append([float(self.qr_data[0].split()[i]),float(self.qr_data[0].split()[i+1])])
                        print('Navigation area x={}, y={}'.format(self.qr_data[1].split()[0],self.qr_data[1].split()[-1]))
                        self.nav.append([float(self.qr_data[1].split()[0]),float(self.qr_data[1].split()[-1])])
                        print('Order number: {}'.format(self.qr_data[-1]))
                        self.order = int(self.qr_data[-1])
                    (x, y, w, h) = bar.rect
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        if self.Color:
            self.color(cv2.inRange(img, self.red_low, self.red_high),'red')          #Red
            self.color(cv2.inRange(img, self.yellow_low, self.yellow_high),'yellow')   #Yellow
            self.color(cv2.inRange(img, self.blue_low, self.blue_high),'blue')          #Blue
        if self.Arrow:
            self.find_arrow(img.copy())
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8")) # ????? ?????? ??? ?????? 1
        except CvBridgeError as e:
            print(e)        
col = ArrowDetecting(True)
#col.Color = True
col.Arrow = True
#col.Qr = True
rospy.sleep(10)

