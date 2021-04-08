# -*- coding: utf-8 -*-
# Инициализация библиотек
import rospy
import time
from clover import srv
from std_srvs.srv import Trigger
# Инициализация класса со всеми функциями
from Main_Config import Main_Config
main = Main_Config()
# Основные данные по заказу и его транспортировке
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

# Засекаем время полета
start_t = time.time()

# Взлетаем с площадки взлета
print navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
# Подлетаем к навигационной стрелке
main.navigate_wait(x=0.8, y=1.2, z=0.5, frame_id='aruco_map')
# Начинаем расознавать положение навигационной стрелки
main.Arrow = True
rospy.sleep(2)

# Летим в секторе, пока не найдем стеллаж
main.sect_fly()
# Проверяем распонали ли мы стеллаж
if main.cx == -1:
    # Если нет, то выводим, что не нашли стеллаж
    print("Dronpoint don't detecting")
    # Производим полет в зону "Старт"
    main.navigate_wait(x=0, y=0, z=0.5, frame_id='aruco_map')
    # Производим посадку
    land()
    # Выводим данные по выполненной задаче
    print("{} don't delivered in {} for {} min {} sec".format(main.order,main.arrow,(time.time()-start_t)//60,(time.time()-start_t)%60))
    # Выходим из программы
    exit()
# Производим посадку на стеллаж, доставку груза
main.land_on_dronpoint()

# Взлетаем со стеллажа
print navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
# Производим полет в зону "Старт"
main.navigate_wait(x=0, y=0, z=0.5, frame_id='aruco_map')

# Производим посадку
land()
# Выводим данные по выполненной задаче
print('{} delivered in {} for {} min {} sec'.format(main.order,main.arrow,(time.time()-start_t)//60,(time.time()-start_t)%60))
