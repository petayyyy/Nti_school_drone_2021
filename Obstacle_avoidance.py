prep = [[1.2, 0],[0.8, 0.4],[1.2, 0.8],[2, 1.6]]
start = [0.4,0.8]
nav = [2, 0.8]
def dop_oblet():
    global prep
    lenn = len(prep)
    for i in range(lenn-1):
        for j in range(i+1,lenn): 
            if i!=j and ((prep[i][0] - prep[j][0])**2 + (prep[i][1] - prep[j][1])**2)**0.5 <= 0.8:  
                prep.append([min(prep[i][0],prep[j][0]) + abs(prep[i][0] - prep[j][0])/2, min(prep[i][1],prep[j][1]) + abs(prep[i][1] - prep[j][1])/2])    
def check(x,y):
    global prep
    if True in [True for i in range(len(prep)) if ((x*0.2-prep[i][0])**2 + (y*0.2-prep[i][1])**2)**(1/2) < 0.4]: return True
    else: return False
dop_oblet()
mas = []
def check_line(a=1,b = 1):
    global v1, v2, nav_x, nav_y, f_x, f_y
    for i in range(17):
        if a == 0:
            if check(nav_x+i,f_y + abs(f_x-nav_x)) == False and nav_x+i <= 12:
                v1 = nav_x+i
                if b == 1: v2 = f_y + abs(f_x-nav_x)
                else: v2 = f_y
                return True
            elif check(nav_x-i,f_y + abs(f_x-nav_x)) == False and nav_x-i >= 0:
                v1 = nav_x-i
                if b == 1: v2 = f_y + abs(f_x-nav_x)
                else: v2 = f_y
                return True
        else:
            if check(f_x + abs(f_y-nav_y),nav_y+i) == False and nav_y+i <= 12:
                v2 = nav_y+i
                if b == 1 : v1 = f_x + abs(f_y-nav_y)
                else: v1 = f_x
                return True
            elif check(f_x + abs(f_y-nav_y),nav_y-i) == False and nav_y-i >= 0:
                v2 = nav_y-i
                if b == 1: v1 = f_x + abs(f_y-nav_y)
                else: v1 = f_x
                return True
        return False
def check_circle():
    global f_x, f_y, v1, v2, mas
    for i in range(17):
        if det_line(f_x+i,f_y,v1,v2) == True and f_x+i <= 16 and det_line(f_x,f_y,f_x+i,f_y) == True:
            mas.append([f_x+i,f_y])
            mas.append([v1,v2])
            f_x, f_y = v1, v2
            return True
        elif det_line(f_x-i,f_y,v1,v2) == True and f_x-i >= 0 and det_line(f_x,f_y,f_x-i,f_y) == True:
            mas.append([f_x-i,f_y])
            mas.append([v1,v2])
            f_x, f_y = v1, v2
            return True
        elif det_line(f_x,f_y+i,v1,v2) == True and f_y+i <= 12 and det_line(f_x,f_y,f_x,f_y+i) == True:
            mas.append([f_x,f_y+i])
            mas.append([v1,v2])
            f_x, f_y = v1, v2
            return True
        elif det_line(f_x,f_y-i,v1,v2) == True and f_y-i >= 0 and det_line(f_x,f_y,f_x,f_y-i) == True:
            mas.append([f_x,f_y-i])
            mas.append([v1,v2])
            f_x, f_y = v1, v2
            return True
    return False
def det_line(x1,y1,x2,y2,r=0.39):
    x1,y1,x2,y2 = x1*0.2,y1*0.2,x2*0.2,y2*0.2
    global prep
    for x,y in prep:
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
def vector_x():
    global v1, v2, nav_x, nav_y
    v1,v2 = nav_x, nav_y
    if check_circle() == False:
        for i in range(12):
            v1,v2 = nav_x+i, nav_y
            if check_circle(): break
            v1,v2 = nav_x-i, nav_y
            if check_circle(): break
def vector_y():
    global v1, v2, nav_x, nav_y
    v1,v2 = nav_x, nav_y
    if check_circle() == False:
        for i in range(12):
            v1,v2 = nav_x, nav_y+i
            if check_circle(): break
            v1,v2 = nav_x, nav_y-i
            if check_circle(): break

def navigate_avoidece(start, finish):
    global nav_x, nav_y, f_x, f_y, v1, v2
    nav_x, nav_y = (finish[0]*100)//20, (finish[1]*100)//20
    f_x, f_y = start[0]//0.2, start[1]//0.2
    h = 0
    while (f_x != nav_x or f_y != nav_y) and h < 5:
        h+=1
        if f_x == nav_x:
            vector_y()
        elif f_y == nav_y:
            vector_x()
        else:
            if nav_x < nav_y:
                check_line(a=0)
            else: 
                check_line(a=1)
            check_circle()
   
navigate_avoidece(start, nav)  
print(mas)
