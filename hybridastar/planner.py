import math, queue, scipy
import numpy as np
from scipy import spatial

import vehicle_lib, grid_a_star, rs_path

XY_GRID_RESOLUTION = 5.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(30.0)  # [rad]
GOAL_TYAW_TH = np.deg2rad(5.0)  # [rad]
MOTION_RESOLUTION = 0.1  # [m] path interporate resolution
N_STEER = 20.0  # number of steer command
EXTEND_AREA = 5.0  # [m] map extend length
SKIP_COLLISION_CHECK = 4  # skip number for collision check

SB_COST = 100.0  # switch back penalty cost
BACK_COST = 5.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
JACKKNIF_COST = 200.0  # Jackknif cost
H_COST = 5.0  # Heuristic cost

WB = vehicle_lib.WB  # [m] Wheel base
MAX_STEER = vehicle_lib.MAX_STEER  # [rad] maximum steering angle

class KDTree:
    def __init__(self, data):
        self.tree = scipy.spatial.cKDTree(data)
    def search_in_distance(self, inp, r):
        index = self.tree.query_ball_point(inp, r)
        return index

class Node(object):
    def __init__(self, xind, yind, yawind, direction, x, y, yaw, directions, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions
        self.steer = steer
        self.cost = cost
        self.pind = pind# parent index

class Config(object):
    def __init__(self, minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww, xyreso, yawreso):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.xyreso = xyreso
        self.yawreso = yawreso
    def prn_obj(obj):
        print ('\n'.join(['%s:%s' % item for item in obj.__dict__.items()]))

def calc_config(ox, oy, xyreso, yawreso):
    min_x_m = min(ox) - EXTEND_AREA
    min_y_m = min(oy) - EXTEND_AREA
    max_x_m = max(ox) + EXTEND_AREA
    max_y_m = max(oy) + EXTEND_AREA
    ox.append(min_x_m)
    oy.append(min_y_m)
    ox.append(max_x_m)
    oy.append(max_y_m)
    minx = round(min_x_m/xyreso)
    miny = round(min_y_m/xyreso)
    maxx = round(max_x_m/xyreso)
    maxy = round(max_y_m/xyreso)
    xw = round(maxx - minx)
    yw = round(maxy - miny)
    minyaw = round(- math.pi/yawreso) - 1
    maxyaw = round(math.pi/yawreso)
    yaww = round(maxyaw - minyaw)
    config = Config(minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww, xyreso, yawreso)
    return config
        
class Path(object):
    def __init__(self, x, y, yaw, direction, cost):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction# direction forward: true, back false
        self.cost = cost

def calc_hybrid_astar_path(sx , sy , syaw , gx , gy , gyaw ,  ox , oy , xyreso , yawreso):
    syaw0 = rs_path.pi_2_pi(syaw)
    gyaw0 = rs_path.pi_2_pi(gyaw)
    global tox,toy
    ox, oy = ox[:], oy[:]
    tox, toy = ox[:], oy[:]
    kdtree = KDTree(np.vstack((tox, toy)).T)
    c = calc_config(ox, oy, xyreso, yawreso)
    nstart = Node(round(sx / xyreso), round(sy / xyreso), round(syaw0 / yawreso), True, [sx], [sy],[syaw0], [True], 0.0, 0.0, -1)
    ngoal = Node(round(gx/xyreso), round(gy/xyreso), round(gyaw0/yawreso), True, [gx],[gy],[gyaw0],[True],0.0,0.0, -1)
    h_dp = grid_a_star.calc_dist_policy(ngoal.x[-1], ngoal.y[-1], ox, oy, xyreso, 1.0)
    openset, closedset = {},{}
    fnode = None
    openset[calc_index(nstart, c)] = nstart
    pq = queue.PriorityQueue()
    pq.put(calc_index(nstart, c),  calc_cost(nstart, h_dp, ngoal, c))
    u, d = calc_motion_inputs()
    nmotion = len(u)
    times = 0
    while 1:
        if len(openset) == 0:
            print ("Error: Cannot find path, No open set")
            return []
        c_id = min(openset, key=lambda o: openset[o].cost + calc_cost(nstart, h_dp, ngoal, c))
        current = openset[c_id]
        # move current node from open to closed
        del openset[c_id]
        closedset[c_id] = current
        isupdated, fpath = update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree)
        if isupdated:  # found
            fnode = fpath
            break
        for i in range(nmotion):
            node = calc_next_node(current, c_id, u[i], d[i], c)
            if verify_index(node, c, ox, oy, kdtree) == False:
                continue
            node_ind = calc_index(node, c)
            # If it is already in the closed set, skip it
            if node_ind in closedset:
                continue
            if not node_ind in openset:
                openset[node_ind] = node
                pq.put(node_ind, calc_cost(nstart, h_dp, ngoal, c))
            else:
                if openset[node_ind].cost > node.cost:
                    # If so, update the node to have a new parent
                    openset[node_ind] = node
            times = times + 1
    print ("final expand node:", len(openset) + len(closedset))
    path = get_final_path(closedset, fnode, nstart, c)
    return path

def calc_index(node, c):
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)
    if ind <= 0:
        print ("Error(calc_index):", ind)
    return ind

def calc_cost(n, h_dp, ngoal, c):
   return (n.cost + 3*H_COST*h_dp[int(n.xind - c.minx)][int(n.yind - c.miny)])

def calc_motion_inputs():
    up = []
    for i in range(int(N_STEER)-1,-1,-1):
        x = MAX_STEER - i*(MAX_STEER/N_STEER)
        up.append(x)
    u = [0.0] + [i for i in up] + [-i for i in up]
    d = [1.0 for i in range(len(u))] + [-1.0 for i in range(len(u))]
    u = u + u
    return u, d

def update_node_with_analystic_expantion(current, ngoal, c, ox, oy, kdtree):
    apath = analystic_expantion(current, ngoal, ox, oy, kdtree)
    if apath != None:
        fx = apath.x[1:]
        fy = apath.y[1:]
        fyaw =  apath.yaw[1:]
        fcost = current.cost + calc_rs_path_cost(apath)
        fpind = calc_index(current, c)
        fd = []
        for d in apath.directions[1:]:
            if d >= 0:
                fd.append(True)
            else:
                fd.append(False)
        fsteer = 0.0
        fpath = Node(current.xind, current.yind, current.yawind, current.direction, fx, fy, fyaw, fd, fsteer, fcost, fpind)
        return True, fpath
    return False, None #no update

def analystic_expantion(n, ngoal, ox, oy, kdtree):
    sx = n.x[-1]
    sy = n.y[-1]
    syaw = n.yaw[-1]
    max_curvature = math.tan(MAX_STEER)/WB
    paths = rs_path.calc_paths(sx,sy,syaw,ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1], max_curvature, step_size=MOTION_RESOLUTION)
    if len(paths) == 0:
        return None
    pathset = {}
    path_id = 0
    for path in paths:
        pathset[path_id] = path
        path_id = path_id + 1
    ind = 0
    for i in range(len(pathset)):
        p_id = min(pathset, key=lambda o: calc_rs_path_cost(pathset[o]))
        path = pathset[p_id]
        if vehicle_lib.check_collision(ox, oy, path.x, path.y, path.yaw, kdtree):
            return path
    return None

def calc_rs_path_cost(rspath):
    cost = 0.0
    for l in rspath.lengths:
        if l >= 0:  # forward
            cost += l
        else:  # back
            cost += abs(l) * BACK_COST
    # swich back penalty
    for i in range(len(rspath.lengths)-1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:  # switch back
            cost += SB_COST
    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S" : # curve
            cost += STEER_COST * abs(MAX_STEER)
    # steer change penalty
    nctypes = len(rspath.ctypes)
    ulist = [0.0 for i in range(nctypes)]
    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER
    for i in range(len(rspath.ctypes)-1):
        cost += STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])
    return cost

def get_final_path(closed, ngoal, nstart, c):
    rx, ry, ryaw = ngoal.x[::-1], ngoal.y[::-1], ngoal.yaw[::-1]
    direction = ngoal.directions[::-1]
    nid = ngoal.pind
    finalcost = ngoal.cost
    while 1:
        n = closed[nid]
        rx.extend(n.x[::-1])
        ry.extend(n.y[::-1])
        ryaw.extend(n.yaw[::-1])
        direction.extend(n.directions[::-1])
        nid = n.pind
        if is_same_grid(n, nstart):
            break
    rx = rx[::-1]
    ry = ry[::-1]
    ryaw = ryaw[::-1]
    direction = direction[::-1]
    # adjuct first direction
    direction[0] = direction[1]
    path = Path(rx, ry, ryaw, direction, finalcost)
    return path

def is_same_grid(node1, node2):
    if node1.xind != node2.xind or node1.yind != node2.yind or node1.yawind != node2.yawind:
        return False
    return True

def calc_next_node(current, c_id, u, d, c):
    arc_l = XY_GRID_RESOLUTION * 1.5
    nlist = math.ceil(arc_l / MOTION_RESOLUTION) + 1
    xlist, ylist, yawlist = [], [], []
    xlist_0 = current.x[-1] + d * MOTION_RESOLUTION * math.cos(current.yaw[-1])
    ylist_0 = current.y[-1] + d * MOTION_RESOLUTION * math.sin(current.yaw[-1])
    yawlist_0 = rs_path.pi_2_pi(current.yaw[-1] + d * MOTION_RESOLUTION / WB * math.tan(u))
    xlist.append(xlist_0)
    ylist.append(ylist_0)
    yawlist.append(yawlist_0)
    for i in range(1,int(nlist)):
        xlist_i = xlist[i-1] + d * MOTION_RESOLUTION * math.cos(yawlist[i-1])
        ylist_i = ylist[i-1] + d * MOTION_RESOLUTION * math.sin(yawlist[i-1])
        yawlist_i = rs_path.pi_2_pi(yawlist[i-1] + d * MOTION_RESOLUTION / WB * math.tan(u))
        xlist.append(xlist_i)
        ylist.append(ylist_i)
        yawlist.append(yawlist_i)
    xind = round(xlist[-1] / c.xyreso)
    yind = round(ylist[-1] / c.xyreso)
    yawind = round(yawlist[-1] / c.yawreso)
    addedcost = 0.0
    if d > 0:
        direction = True
        addedcost += abs(arc_l)
    else:
        direction = False
        addedcost += abs(arc_l) * BACK_COST
    # swich back penalty
    if direction != current.direction:  # switch back penalty
        addedcost += SB_COST
    # steer penalyty
    addedcost += STEER_COST * abs(u)
    # steer change penalty
    addedcost += STEER_CHANGE_COST * abs(current.steer - u)
    cost = current.cost + addedcost
    directions = [direction for i in range(len(xlist))]
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, directions, u, cost, c_id)
    return node

def verify_index(node, c, ox, oy, kdtree):
    if (node.xind - c.minx) >= c.xw or (node.xind - c.minx) <= 0 or (node.yind - c.miny) >= c.yw or (node.yind - c.miny) <= 0:
        return False
    if vehicle_lib.check_collision(ox, oy, node.x, node.y, node.yaw, kdtree) == False:
        return False
    return True
