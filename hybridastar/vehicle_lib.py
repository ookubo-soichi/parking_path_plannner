import math
import numpy as np

# Vehicle parameter
W = 1.8 #[m] width of vehicle
LF = 3.7 #[m] distance from rear to vehicle front end of vehicle
LB = 1.0 #[m] distance from rear to vehicle back end of vehicle
TR = 0.5 # Tyre radius [m] for plot
TW = 1.0 # Tyre width [m] for plot
MAX_STEER = 0.6 #[rad] maximum steering angle
WB = 2.7  #[m] wheel base: rear to front steer

# for collision check
cW = 2.5 # width of vehicle
WBUBBLE_DIST = (LB+LF)/2.0-LB
WBUBBLE_R = (LB+LF)/2.0

def vehicle_outline(x=0, y=0, yaw=0, steer=0):
    LENGTH = LB+LF
    truckOutLine = np.array([[-LB, (LENGTH - LB), (LENGTH - LB), (-LB), (-LB)], [W / 2, W / 2, - W / 2, - W / 2, W / 2]])
    rr_wheel = np.array([[TR, - TR, - TR, TR, TR], [-W / 12.0 + TW, - W / 12.0 + TW, W / 12.0 + TW, W / 12.0 + TW, - W / 12.0 + TW]])
    rl_wheel = np.array([[TR, - TR, - TR, TR, TR], [-W / 12.0 - TW, - W / 12.0 - TW, W / 12.0 - TW, W / 12.0 - TW, - W / 12.0 - TW]])
    fr_wheel = np.array([[TR, - TR, - TR, TR, TR], [- W / 12.0 + TW, - W / 12.0 + TW, W / 12.0 + TW, W / 12.0 + TW, - W / 12.0 + TW]])
    fl_wheel = np.array([[TR, - TR, - TR, TR, TR], [-W / 12.0 - TW, - W / 12.0 - TW, W / 12.0 - TW, W / 12.0 - TW, - W / 12.0 - TW]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])
    fr_wheel = np.dot(fr_wheel.T, Rot2).T
    fl_wheel = np.dot(fl_wheel.T, Rot2).T
    fr_wheel[0,:] += WB
    fl_wheel[0,:] += WB
    fr_wheel = np.dot(fr_wheel.T, Rot1).T
    fl_wheel = np.dot(fl_wheel.T, Rot1).T
    truckOutLine = np.dot(truckOutLine.T, Rot1)
    rr_wheel = np.dot(rr_wheel.T, Rot1).T
    rl_wheel = np.dot(rl_wheel.T, Rot1).T
    truckOutLine = truckOutLine.T
    truckOutLine[0,:] += x
    truckOutLine[1,:] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y
    return truckOutLine, fr_wheel, rr_wheel, fl_wheel, rl_wheel

def check_collision(ox, oy, x, y, yaw, kdtree):
    for (ix, iy, iyaw) in zip(x, y, yaw):
        cx = ix + WBUBBLE_DIST* math.cos(iyaw)
        cy = iy + WBUBBLE_DIST* math.sin(iyaw)
        ids = kdtree.search_in_distance([cx,cy], WBUBBLE_R)
        if len(ids) == 0:
            continue
        vrx = [LF, LF, -LB, -LB, LF]
        vry = [-cW / 2.0, cW / 2.0, cW / 2.0, -cW / 2.0, -cW / 2.0]
        c = math.cos(iyaw)
        s = math.sin(iyaw)
        for _id in ids:
            tx = ox[_id] - ix
            ty = oy[_id] - iy
            lx = (c*tx + s*ty)
            ly = (-s*tx + c*ty)
            sumangle = 0.0
            for i in range(len(vrx)-1):
                x1 = vrx[i] - lx
                y1 = vry[i] - ly
                x2 = vrx[i+1] - lx
                y2 = vry[i+1] - ly
                d1 = math.hypot(x1,y1)
                d2 = math.hypot(x2,y2)
                theta1 = math.atan2(y1,x1)
                tty = (-math.sin(theta1)*x2 + math.cos(theta1)*y2)
                tmp = (x1*x2+y1*y2)/(d1*d2)
                tmp = max(min(tmp, 1.0), 0.0)
                if tty >= 0.0:
                    sumangle += math.acos(tmp)
                else:
                    sumangle -= math.acos(tmp)
            if sumangle >= math.pi:
                return False
    return True

