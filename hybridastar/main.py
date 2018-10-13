import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import planner, vehicle_lib

# set obstacles
def make_ob(wx=25, topy=12, btmy=-5, wp=3):
    ox = []
    oy = []
    for i in range(-wx, wx+1):
        ox.append(i)
        oy.append(topy)
    for i in range(-wx, -wp):
        ox.append(i)
        oy.append(4.0)
    for i in range(btmy, 5):
        ox.append(-wp)
        oy.append(i)
    for i in range(btmy, 5):
        ox.append(wp)
        oy.append(i)
    for i in range(wp, wx):
        ox.append(i)
        oy.append(4.0)
    for i in range(-wp, wp):
        ox.append(i)
        oy.append(btmy)
    for i in range(4, topy):
        ox.append(-wx)
        oy.append(i)
    for i in range(4, topy):
        ox.append(wx)
        oy.append(i)
    return ox, oy

def plot_vehicle(ax, x, y, yaw):
    ol, frw, rrw, flw, rlw = vehicle_lib.vehicle_outline(x,y,yaw,0)
    ax.plot(x, y, "*")
    ax.plot(frw[0, :], frw[1, :], 'k-')
    ax.plot(rrw[0, :], rrw[1, :], 'k-')
    ax.plot(flw[0, :], flw[1, :], 'k-')
    ax.plot(rlw[0, :], rlw[1, :], 'k-')
    ax.plot(ol[0, :], ol[1, :], 'k-')
    
def main():
    s = [-14, 7, 0]
    g = [0, 0, math.pi/2]
    ox, oy = make_ob()
    path = planner.calc_hybrid_astar_path(s[0],s[1],s[2], g[0],g[1],g[2], ox, oy, planner.XY_GRID_RESOLUTION, planner.YAW_GRID_RESOLUTION)
    fig, ax = plt.subplots()
    plt.axis("equal")
    plt.subplots_adjust(bottom=0.35)
    ax_x = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    ax_y = plt.axes([0.1, 0.20, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    ax_yaw = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    ax_index = plt.axes([0.1, 0.10, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    s_x = Slider(ax_x, 'x', -20.0, 20.0, valinit=s[0])
    s_y = Slider(ax_y, 'y', 5.0, 10.0, valinit=s[1])
    s_yaw = Slider(ax_yaw, 'yaw', -math.pi, math.pi, valinit=s[2])
    s_index = Slider(ax_index, 'index', 0, 1.0, valinit=0)
    ax.plot(ox,oy,'ko')
    plot_vehicle(ax, s[0], s[1], s[2])
    plot_vehicle(ax, g[0], g[1], g[2])
    def update_index(val):
        global path
        ax.cla()
        ax.plot(ox,oy,'ko')
        x = s_x.val
        y = s_y.val
        yaw=s_yaw.val
        index=s_index.val
        plot_vehicle(ax, x, y, yaw)
        plot_vehicle(ax, g[0], g[1], g[2])
        _idx = max(min(int(len(path.x) * index), len(path.x)-1), 0)
        plot_vehicle(ax, path.x[_idx], path.y[_idx], path.yaw[_idx])
        ax.plot(path.x, path.y, 'r-')
        fig.canvas.draw_idle()
    def update(val):
        global path
        ax.cla()
        x = s_x.val
        y = s_y.val
        yaw=s_yaw.val
        index=s_index.val
        ax.plot(ox,oy,'ko')
        plot_vehicle(ax, x, y, yaw)
        plot_vehicle(ax, g[0], g[1], g[2])
        path = planner.calc_hybrid_astar_path(x,y,yaw, g[0],g[1],g[2], ox, oy, planner.XY_GRID_RESOLUTION, planner.YAW_GRID_RESOLUTION)
        ax.plot(path.x, path.y, 'r-')
        fig.canvas.draw_idle()
    s_x.on_changed(update)
    s_y.on_changed(update)
    s_yaw.on_changed(update)
    s_index.on_changed(update_index)
    plt.show()

main()

