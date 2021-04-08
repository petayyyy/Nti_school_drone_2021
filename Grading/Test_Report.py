# -*- coding: utf-8 -*-
from tkinter import *
root=Tk()
root.title('Test_Report')

# Сюда подставляете координаты препятствий
prep = [] # Сюда подставляете координаты препятствий
# Сюда подставляете координаты навигационной стрелки
mas = [] 

canv=Canvas(root,width=5*80,height=4*80,bg="white",cursor="pencil")

for i in range(len(prep)): canv.create_oval(prep[i][0]*100+20,prep[i][1]*100+20,prep[i][0]*100-20,prep[i][1]*100-20,fill="red",outline="red")

f, median_prep, x_last, y_last = open('Report.txt', 'r'),[],0,0
flag = [False]*len(prep)
for i in range(len(prep)-1):
    for j in range(i+1,len(prep)):
        if i!=j and ((prep[i][0] - prep[j][0])**2 + (prep[i][1] - prep[j][1])**2)**0.5 <= 0.8:  
            median_prep.append([min(prep[i][0],prep[j][0]) + abs(prep[i][0] - prep[j][0])/2, min(prep[i][1],prep[j][1]) + abs(prep[i][1] - prep[j][1])/2, i, j])
for x,y in mas:
    x, y = map(float,input().split(' '))
    canv.create_polygon([x*100,y*100],[x_last*100,y_last*100],fill="blue",outline="blue")        
    #print(x, y)
    for i in range (len(prep)):
        if ((x-prep[i][0])**2 + (y-prep[i][1])**2)**(1/2) < 0.4: 
            flag[i] = True
    for i in range (len(median_prep)):
        if ((x-median_prep[i][0])**2 + (y-median_prep[i][1])**2)**(1/2) < 0.4: 
            flag[median_prep[i][-1]], flag[median_prep[i][-2]] = True, True
            print('Читайте регламент внимательней')
    x_last, y_last = x,y

print('\n'.join(['Препятствие {} побито'.format(i+1) if flag[i] == True else 'Ну хоть {} не задели'.format(i+1) for i in range(len(flag))]))
print('Ваш балл за это задание равен {}'.format(sum([0 if flag[i] == True else 3 for i in range(len(flag))])))

canv.pack()

# здесь будет будущий код
root.mainloop()
