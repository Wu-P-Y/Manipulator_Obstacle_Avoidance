'''
参考:https://github.com/redglassli/PythonRobotics/blob/master/PathPlanning/RRT/rrt.py

平面内的快速随机搜索树法路径规划
'''

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True

class RRT:

    class Node:
        '''
        节点Node
        '''
        def __init__(self, x, y) -> None:
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, obstacles, rand_area,
                expand_dis = 3.0, path_reso = 0.5, goal_sample_rate = 5, max_iter = 500) -> None:
        '''
        @start 起点
        @goal 目标
        @obstacles 障碍物集合 [[x, y, size],...]
        @rand_area 随机采样区域 [min, max]
        @expand_dis 拓展长度
        @path_reso 路径单位长度
        @goal_sample_rate 目标采样率
        @max_iter 最大迭代次数
        '''
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_reso = path_reso
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacles = obstacles
        self.node_list = []

    def planning(self, animation = True):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacles):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dis_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:        # 如果当前节点到目标的距离已经小于一个拓展长度
                final_node = self.steer(self.node_list[-1], self.goal, self.expand_dis)
                if self.check_collision(final_node, self.obstacles):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None     # 寻路失败

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:  
            rnd = self.Node(self.goal.x, self.goal.y)
        return rnd

    def steer(self, begin:Node, end:Node, extend_length = float("inf")):
        '''
        @begin 当前节点
        @end 随机生成的下一节点
        @extend_length 前进的长度
        '''
        new_node = self.Node(begin.x, begin.y)
        d, theta = self.calc_distance_and_angle(new_node, end)

        # 从new_node出发寻路，用path_x和path_y保存路径
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expend = math.floor(extend_length / self.path_reso)

        for _ in range(n_expend):
            # 前进一个路径单位长度
            new_node.x += self.path_reso * math.cos(theta)
            new_node.y += self.path_reso * math.sin(theta)
            # 将点放入路径list
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, end)
        if d <= self.path_reso:     # 最后一段距离小于一个路径单位长度
            # 将下一节点加入路径中
            new_node.path_x.append(end.x)
            new_node.path_y.append(end.y)

        new_node.parent = begin     # 起点为当前节点的父节点

        return new_node

    def generate_final_course(self, goal_ind):
        '''
        @goal_ind 终点在路径中的索引
        '''
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dis_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        
        return np.linalg.norm([dx, dy])


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = np.argmin(dlist)

        return minind

    @staticmethod
    def calc_distance_and_angle(begin:Node, end:Node):
        dis_x = end.x - begin.x
        dis_y = end.y - begin.y
        dis = np.linalg.norm([dis_x, dis_y])
        theta = math.atan2(dis_y, dis_x)
        return dis, theta

    @staticmethod
    def check_collision(node:Node, obstacles:list):
        '''
        True 无碰撞
        Fasle 有碰撞
        '''
        if node is None:
            return False

        for (ox, oy, size) in obstacles:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False 
        
        return True

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def draw_graph(self, rnd = None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacles:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


def main(goal_x = 6.0, goal_y = 10.0):
    print("start " + __file__)

    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (7, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    
    rrt = RRT(start=[0, 0],
              goal=[goal_x, goal_y],
              rand_area=[-2, 15],
              obstacles=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
    

if __name__ == '__main__':
    main()