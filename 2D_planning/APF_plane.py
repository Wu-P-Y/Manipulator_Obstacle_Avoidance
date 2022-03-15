'''
参考:https://github.com/redglassli/PythonRobotics/blob/master/PathPlanning/PotentialFieldPlanning/potential_field_planning.py

平面内的人工势场法路径规划
'''

from cmath import inf
from importlib.resources import path
import numpy as np
import matplotlib.pyplot as plt

KP = 5.0    # 引力势场增益
ETA = 100.0 # 斥力势场增益
AREA_WIDTH = 30.0   # 地图宽度(假设地图为正方形)

show_animation = True

class point():
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

class potential_field():
    def __init__(self, goal:point, obstacles:list, reso:float, rr:float) -> None:
        '''
        @goal: 目标点
        @obstacles: 障碍物点集
        @reso: 分辨率，即最小划分单元格边长
        @rr: 机器人半径
        '''
        ox, oy = get_obstacle_axis(obstacles)

        minx = min(ox) - AREA_WIDTH / 2.0      # 指定地图边界
        miny = min(oy) - AREA_WIDTH / 2.0
        maxx = max(ox) + AREA_WIDTH / 2.0
        maxy = max(oy) + AREA_WIDTH / 2.0

        units_x = int(round((maxx - minx) / reso))
        units_y = int(round((maxy - miny) / reso))

        # 计算每个单元格的势场
        pmap = [[0.0 for i in range(units_y)] for i in range(units_x)]

        for unitx in range(units_x):
            x = unitx * reso + minx

            for unity in range(units_y):
                y = unity * reso + miny
                p = point(x, y)
                a_p = attractive_potential(p, goal)        # 引力
                r_p = repulsive_potential(p, obstacles, rr)     # 斥力
                potential = a_p + r_p
                pmap[unitx][unity] = potential

        self.pmap = pmap
        self.minx = minx
        self.miny = miny


def get_obstacle_axis(obstacles:list):
    '''
    @obstacles: 障碍物点集

    获取每个障碍物点的 x, y 坐标
    '''
    ox = []
    oy = []
    for obstacle in obstacles:      
        ox.append(obstacle.x)
        oy.append(obstacle.y)
    return ox, oy


def attractive_potential(point:point, goal:point):
    '''
    @point: 当前单元格位置
    @goal: 目标点位置

    计算当前单元格的引力
    '''
    return 0.5 * KP * np.hypot(point.x - goal.x, point.y - goal.y)


def repulsive_potential(point:point, obstacles:list, rr:float):
    '''
    @point: 当前单元格位置
    @obstacles: 障碍物点集
    @rr: 机器人半径

    计算当前单元格的斥力
    '''
    ox, oy = get_obstacle_axis(obstacles)

    # 寻找最近的障碍物
    closest_index = -1          # 最近障碍物下标
    closest_dis = float(inf)      # 最近障碍物距离
    for i, _ in enumerate(ox):
        dis = np.hypot(point.x - ox[i], point.y - oy[i])
        if closest_dis >= dis:
            closest_dis = dis
            closest_index = i

    # 计算斥力
    dq = np.hypot(point.x - ox[closest_index], point.y - oy[closest_index])

    if dq <= rr:        # 如果有碰撞风险
        if dq <= 0.1:
            dq = 0.1

        return 0.8 * ETA * (1.0/ dq - 1.0 / rr) ** 2
    else:       #如果没有碰撞风险
        return 0.0


def next_step():
    '''
    机器人的运动方向
    '''
    next_step = [
        [1, 0],
        [0, 1],
        [-1, 0],
        [0, -1],
        [-1, -1],
        [-1, 1],
        [1, -1],
        [1, 1]
    ]

    return next_step

# ------------------------------------------------------------------------------ #

def path_planning(start:point, goal:point, obstacles:list, reso:float, rr:float):
    p_f = potential_field(goal, obstacles, reso, rr)
    pmap = p_f.pmap
    minx = p_f.minx
    miny = p_f.miny

    dis = np.hypot(start.x - goal.x, start.y - goal.y)
    # 获取start点和goal点在map中的索引
    six = round((start.x - minx) / reso)
    siy = round((start.y - miny) / reso)
    gix = round((goal.x - minx) / reso)
    giy = round((goal.y - miny) / reso)


    ix = six
    iy = siy

    # 画图
    if show_animation:
        draw_heatmap(pmap)
        # press 'Esc' to stop
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(six, siy, "*k")
        plt.plot(gix, giy, "*m")
        plt.pause(1)

    path = [point(start.x, start.y)]        # list path stores the points of path
    motion = next_step()
    while dis >= reso and len(path) < 120:       # 循环直到机器人到达终点
        min_i = float("inf")       # min_i存放所有下一步中的最小势场
        minx, miny = -1, -1       # minx, miny存放坐标
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):       # 如果超出了map边界
                p = float("inf")
            else:
                p = pmap[inx][iny]
            if min_i > p:
                min_i = p
                minx = inx
                miny = iny
        ix = minx
        iy = miny
        xp = ix * reso + minx
        yp = iy * reso + miny
        dis = np.hypot(goal.x - xp, goal.y - yp)
        path.append(point(xp, yp))
        # print(path[-1]) 

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    return path


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

# ------------------------------------------------------------------------------ #

def main():
    print("potential_field_planning start")

    start = point(0.0, 10.0)
    goal = point(30.0, 30.0)

    reso = 0.5
    robot_radius = 10.0

    obstacles = [point(15.0, 25.0), point(5.0, 15.0), point(20.0, 28.0), point(25.0, 25.0), point(5.0, 15.5), point(5.0, 14.5)]
    path = path_planning(start, goal, obstacles, reso, robot_radius)

    for p in path:
        print(p.x, ",", p.y)

    if show_animation:
        plt.show()



if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")