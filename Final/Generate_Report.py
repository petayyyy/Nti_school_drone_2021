# -*- coding: utf-8 -*-
# Обязательная конструкция для работы с кириллицей

# Инициализация библиотек
import rospy
from clover import srv
from std_msgs.msg import Float32MultiArray

# Инициализация ноды
rospy.init_node('fly')
# Инициализация сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

# Создаем файл для записи данных
f.open('Report.txt', 'w')
# Пока программа работает
while not rospy.is_shutdown():
    # Берем данные коптера(его координаты положения в простраестве относительно frame_id = 'aruco_map')
    telem = get_telemetry(frame_id = 'aruco_map')
    # Записываем эти данные в файл, в соответствии с требованиями регламента
    f.write('{} {}  {}\n'.format(telem.x, telem.y, telem.z))
    # Выводим в терминал эти данные
    print('{} {}  {}'.format(telem.x, telem.y, telem.z))
    # Производим задержку, для ограничения количества данных
    rospy.sleep(0.1)
