# -*- coding: utf-8 -*-
import rospy
import time
from clover import srv
from std_srvs.srv import Trigger
from Main_Config import Main_Config
main = Main_Config()
main.zz = 0.5
main.order = 3
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

start_t = time.time()

print navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
main.navigate_wait(x=0.8, y=1.2, z=0.5, frame_id='aruco_map')
main.Arrow = True
rospy.sleep(2)

main.sect_fly()
main.land_on_dronpoint()

print navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)

main.navigate_wait(x=0, y=0, z=0.5, frame_id='aruco_map')
land()
print('{} delivered in {} for {} min {} sec'.format(main.order,main.arrow,(time.time()-start_t)//60,(time.time()-start_t)%60))
