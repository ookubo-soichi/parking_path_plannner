import math, queue

class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

def calc_dist_policy(gx, gy, ox, oy, reso, vr):
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]
    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, vr)
    openset, closedset = {}, {}
    openset[calc_index(ngoal, xw, minx, miny)] = ngoal
    motion = get_motion_model()
    nmotion = len(motion)
    pq = queue.PriorityQueue()
    pq.put(calc_index(ngoal, xw, minx, miny), ngoal.cost)
    while True:
        if len(openset) == 0:
            break
        c_id = pq.get()
        current = openset[c_id]
        del openset[c_id]
        closedset[c_id] = current
        for i in range(nmotion):
            node = Node(current.x+motion[i][0], current.y+motion[i][1], current.cost+motion[i][2], c_id)
            if verify_node(node, minx, miny, xw, yw, obmap) == False:
                continue
            node_ind = calc_index(node, xw, minx, miny)
            if node_ind in closedset:
                continue
            if node_ind in openset:
                if openset[node_ind].cost > node.cost:
                    openset[node_ind].cost = node.cost
                    openset[node_ind].pind = c_id
            else:
                openset[node_ind] = node
                pq.put(calc_index(node, xw, minx, miny), node.cost)
    pmap = calc_policy_map(closedset, xw, yw, minx, miny)
    return pmap

def calc_obstacle_map(ox, oy, reso, vr):
    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    obmap = [[False for i in range(int(ywidth))] for i in range(int(xwidth))]
    for ix in range(int(xwidth)):
        x = ix + minx
        for iy in range(int(ywidth)):
            y = iy + miny
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
    return obmap, minx, miny, maxx, maxy, xwidth, ywidth

def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)

def get_motion_model():
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]
    return motion

def verify_node(node, minx, miny, xw, yw, obmap):
    if (node.x - minx) >= xw or (node.x - minx) <= 0 or (node.y - miny) >= yw or (node.y - miny) <= 0:
        return False
    if obmap[int(node.x-minx)][int(node.y-miny)]:
        return False
    return True

def calc_policy_map(closedset, xw, yw, minx, miny):
    pmap = [[False for i in range(int(yw))] for i in range(int(xw))]
    for v in closedset.values():
        pmap[int(v.x-minx)][int(v.y-miny)] = v.cost
    return pmap
