# -*- coding: utf-8 -*-
# Обязательная конструкция для работы с кириллицей

# Инициализация библиотек
import rospy
import time
from clover import srv
from std_srvs.srv import Trigger
# Инициализация класса со всеми функциями
from Main_Config import Main_Config
main = Main_Config()
# Основные данные по транспортировке заказа
main.zz = 0.75

# Инициализация сервисов
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
# Подлетаем к Qr коду
main.navigate_wait(x=0.4, y=0.8, z=0.5, frame_id='aruco_map')
# Начинаем распознавать содержание Qr кода
main.Qr = True
rospy.sleep(2)
# Проверяем распонали ли мы содержание Qr кода
if main.order == -1:
    # Если нет, то пробуем еще раз распознавать содержание Qr кода, но на другой высоте
    main.Qr = True
    main.navigate_wait(x=0.4, y=0.8, z=0.7, frame_id='aruco_map')
    # Проверяем распонали ли мы содержание Qr кода
    if main.order == -1: 
        # Если не распонали содержание Qr кода, то выводим что его не нашли
        print("Qr don't detecting")
        # Производим полет в зону "Старт"
        main.navigate_wait(x=0, y=0, z=0.5, frame_id='aruco_map')
        # Производим посадку
        land()
        # Выходим из программы
        exit()
# Подлетаем к навигационной стрелке, облетая препятствия, записанные в Qr коде
main.navigate_avoidece([0.4,0.8], main.nav)
main.navigate_mas()
main.navigate_wait(x=main.nav[0], y=main.nav[1], z=0.5, frame_id='aruco_map')
# Начинаем распознавать положение навигационной стрелки
main.Arrow = True
rospy.sleep(2)

# Летим в секторе, пока не найдем стеллаж
main.sect_fly()
rospy.sleep(20)
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
